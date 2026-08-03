"""Optional LeRobot v3 recording for bimanual YAM teleoperation."""

from __future__ import annotations

import math
import shutil
import sys
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from queue import Full, Queue
from threading import Lock, Thread
from typing import Any, Literal, Protocol

import numpy as np

from umi_teleop.controller import ARM_NAMES

CAMERA_NAMES = ("top", "left", "right")
CAMERA_HEIGHT = 360
CAMERA_WIDTH = 640
CAMERA_SHAPE = (CAMERA_HEIGHT, CAMERA_WIDTH, 3)
TOP_CAMERA_CAPTURE_HEIGHT = 480
DATASET_FPS = 30
YAM_SCALAR_NAMES = (
    *(f"left_joint_{index}.pos" for index in range(6)),
    "left_gripper.pos",
    *(f"right_joint_{index}.pos" for index in range(6)),
    "right_gripper.pos",
)


@dataclass(frozen=True)
class DatasetConfig:
    """Opt-in LeRobot dataset settings for ``umi-yam``."""

    repo_id: str | None = None
    root: Path | None = None
    task: str | None = None
    top_camera_serial: str | None = None
    left_camera_serial: str | None = None
    right_camera_serial: str | None = None
    resume: bool = False
    push_to_hub: bool = False
    private: bool = True
    min_episode_frames: int = 15
    queue_size: int = 300
    camera_max_age_ms: int = 250
    max_control_lag_s: float = 0.10
    min_free_disk_gb: float = 5.0
    image_writer_threads_per_camera: int = 2
    encoder_threads: int = 2
    video_codec: Literal["h264", "libsvtav1"] = "h264"
    shutdown_timeout_s: float = 60.0

    @property
    def enabled(self) -> bool:
        return self.repo_id is not None

    @property
    def camera_serials(self) -> dict[str, str]:
        values = {
            "top": self.top_camera_serial,
            "left": self.left_camera_serial,
            "right": self.right_camera_serial,
        }
        return {name: value.strip() for name, value in values.items() if value is not None}

    def validate(self, control_hz: float) -> None:
        if not self.enabled:
            return
        repo_id = (self.repo_id or "").strip()
        if repo_id.count("/") != 1 or any(not part for part in repo_id.split("/")):
            raise DatasetRecordingError("dataset repo ID must be OWNER/NAME")
        if not self.task or not self.task.strip():
            raise DatasetRecordingError("dataset task must be nonempty")
        if sys.version_info < (3, 12):
            raise DatasetRecordingError("dataset recording requires Python 3.12")
        if not math.isclose(control_hz, DATASET_FPS, rel_tol=0.0, abs_tol=1e-9):
            raise DatasetRecordingError(
                f"dataset recording requires a {DATASET_FPS} Hz YAM control loop"
            )
        serials = self.camera_serials
        missing = [name for name in CAMERA_NAMES if name not in serials or not serials[name]]
        if missing:
            raise DatasetRecordingError(
                "dataset recording requires camera serials for " + ", ".join(missing)
            )
        if len(set(serials.values())) != len(CAMERA_NAMES):
            raise DatasetRecordingError("top, left, and right camera serials must be distinct")
        if self.resume:
            if self.root is None:
                raise DatasetRecordingError("dataset resume requires an explicit local root")
            if not (self.root / "meta" / "info.json").is_file():
                raise DatasetRecordingError(
                    f"dataset resume root is missing meta/info.json: {self.root}"
                )
        elif self.root is not None and self.root.exists():
            raise DatasetRecordingError(
                f"new dataset root already exists; use a new path or --dataset.resume: {self.root}"
            )
        for name, value in (
            ("min episode frames", self.min_episode_frames),
            ("queue size", self.queue_size),
            ("camera max age", self.camera_max_age_ms),
            ("image writer threads per camera", self.image_writer_threads_per_camera),
            ("encoder threads", self.encoder_threads),
        ):
            if value <= 0:
                raise DatasetRecordingError(f"dataset {name} must be positive")
        if self.max_control_lag_s <= 0.0:
            raise DatasetRecordingError("dataset max control lag must be positive")
        if self.min_free_disk_gb <= 0.0:
            raise DatasetRecordingError("dataset minimum free disk space must be positive")
        if self.shutdown_timeout_s <= 0.0:
            raise DatasetRecordingError("dataset shutdown timeout must be positive")


class DatasetRecordingError(RuntimeError):
    pass


class DatasetFrameError(DatasetRecordingError):
    """A recoverable frame/camera error that invalidates only the active episode."""


class Camera(Protocol):
    def read_latest(self, max_age_ms: int = 500) -> np.ndarray: ...

    def disconnect(self) -> None: ...


class Dataset(Protocol):
    meta: Any

    def add_frame(self, frame: dict[str, Any]) -> None: ...

    def save_episode(self) -> None: ...

    def clear_episode_buffer(self, delete_images: bool = True) -> None: ...

    def finalize(self) -> None: ...

    def push_to_hub(self, **kwargs: Any) -> None: ...


DatasetFactory = Callable[[DatasetConfig, dict[str, dict[str, Any]]], Dataset]
CameraFactory = Callable[[DatasetConfig], dict[str, Camera]]


def dataset_features() -> dict[str, dict[str, Any]]:
    features: dict[str, dict[str, Any]] = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(YAM_SCALAR_NAMES),),
            "names": list(YAM_SCALAR_NAMES),
        },
        "action": {
            "dtype": "float32",
            "shape": (len(YAM_SCALAR_NAMES),),
            "names": list(YAM_SCALAR_NAMES),
        },
    }
    for name in CAMERA_NAMES:
        features[f"observation.images.{name}"] = {
            "dtype": "video",
            "shape": CAMERA_SHAPE,
            "names": ["height", "width", "channels"],
            "info": {"is_depth_map": False},
        }
    return features


def yam_vector(
    joints: Mapping[str, np.ndarray], grippers: Mapping[str, float], *, label: str
) -> np.ndarray:
    if set(joints) != set(ARM_NAMES) or set(grippers) != set(ARM_NAMES):
        raise DatasetFrameError(f"{label} must contain left and right")
    values: list[float] = []
    for name in ARM_NAMES:
        arm_joints = np.asarray(joints[name], dtype=np.float64)
        if arm_joints.shape != (6,) or not np.isfinite(arm_joints).all():
            raise DatasetFrameError(f"{label} {name} joints must contain six finite values")
        grip = float(grippers[name])
        if not math.isfinite(grip) or not 0.0 <= grip <= 1.0:
            raise DatasetFrameError(f"{label} {name} gripper must be normalized to [0, 1]")
        values.extend(arm_joints.tolist())
        values.append(grip)
    return np.asarray(values, dtype=np.float32)


def build_frame(
    *,
    images: Mapping[str, np.ndarray],
    measured_joints: Mapping[str, np.ndarray],
    measured_grippers: Mapping[str, float],
    commanded_joints: Mapping[str, np.ndarray],
    commanded_grippers: Mapping[str, float],
    task: str,
) -> dict[str, Any]:
    if set(images) != set(CAMERA_NAMES):
        raise DatasetFrameError("camera frames must contain top, left, and right")
    frame: dict[str, Any] = {
        "observation.state": yam_vector(measured_joints, measured_grippers, label="measured state"),
        "action": yam_vector(commanded_joints, commanded_grippers, label="commanded action"),
        "task": task,
    }
    for name in CAMERA_NAMES:
        image = np.asarray(images[name])
        if image.shape != CAMERA_SHAPE:
            raise DatasetFrameError(
                f"{name} camera frame shape {image.shape} does not match {CAMERA_SHAPE}"
            )
        if image.dtype != np.uint8:
            raise DatasetFrameError(f"{name} camera frame must be RGB uint8")
        frame[f"observation.images.{name}"] = np.ascontiguousarray(image).copy()
    return frame


def normalize_camera_image(name: str, image: np.ndarray) -> np.ndarray:
    """Return one RGB camera image in the fixed LeRobot dataset shape."""
    image = np.asarray(image)
    if name == "top" and image.shape == (TOP_CAMERA_CAPTURE_HEIGHT, CAMERA_WIDTH, 3):
        crop_start = (TOP_CAMERA_CAPTURE_HEIGHT - CAMERA_HEIGHT) // 2
        image = image[crop_start : crop_start + CAMERA_HEIGHT]
    if image.shape != CAMERA_SHAPE:
        raise DatasetFrameError(
            f"{name} camera frame shape {image.shape} cannot be converted to {CAMERA_SHAPE}"
        )
    if image.dtype != np.uint8:
        raise DatasetFrameError(f"{name} camera frame must be RGB uint8")
    return np.ascontiguousarray(image).copy()


def _normalized_features(features: Mapping[str, Mapping[str, Any]]) -> dict[str, tuple[Any, ...]]:
    return {
        key: (
            feature.get("dtype"),
            tuple(feature.get("shape", ())),
            tuple(feature.get("names") or ()),
        )
        for key, feature in features.items()
    }


def _features_match(
    actual: Mapping[str, Mapping[str, Any]], expected: Mapping[str, Mapping[str, Any]]
) -> bool:
    default_keys = {"timestamp", "frame_index", "episode_index", "index", "task_index"}
    actual_normalized = _normalized_features(actual)
    expected_normalized = _normalized_features(expected)
    if set(actual_normalized) - default_keys != set(expected_normalized):
        return False
    return all(actual_normalized.get(key) == value for key, value in expected_normalized.items())


def _default_dataset_factory(config: DatasetConfig, features: dict[str, dict[str, Any]]) -> Dataset:
    try:
        from lerobot.configs import RGBEncoderConfig
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise DatasetRecordingError(
            "LeRobot recording dependencies are unavailable; install the dataset extra"
        ) from exc

    common = {
        "root": config.root,
        "image_writer_processes": 0,
        "image_writer_threads": config.image_writer_threads_per_camera * len(CAMERA_NAMES),
        "batch_encoding_size": 1,
        "encoder_threads": config.encoder_threads,
        "rgb_encoder": RGBEncoderConfig(vcodec=config.video_codec, pix_fmt="yuv420p"),
    }
    if config.resume:
        dataset = LeRobotDataset.resume(config.repo_id, **common)
        actual = getattr(dataset.meta, "features", {})
        if not _features_match(actual, features) or dataset.meta.fps != DATASET_FPS:
            dataset.finalize()
            raise DatasetRecordingError(
                "existing dataset schema or frame rate does not match the 30 Hz "
                "MolmoAct2 Bimanual YAM contract"
            )
        return dataset
    return LeRobotDataset.create(
        config.repo_id,
        DATASET_FPS,
        features=features,
        robot_type="bi_yam_follower",
        use_videos=True,
        **common,
    )


def _default_upload_validator(config: DatasetConfig, dataset: Dataset) -> None:
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        raise DatasetRecordingError("cannot validate finalized dataset without LeRobot") from exc
    root = getattr(dataset, "root", config.root)
    reopened = LeRobotDataset(
        config.repo_id,
        root=root,
        download_videos=False,
        video_backend="pyav",
        return_uint8=True,
    )
    validate_finalized_dataset(reopened)
    for frame_index in sorted({0, reopened.meta.total_frames - 1}):
        sample = reopened[frame_index]
        for name in CAMERA_NAMES:
            image = sample[f"observation.images.{name}"]
            if tuple(image.shape) not in (CAMERA_SHAPE, (3, CAMERA_HEIGHT, CAMERA_WIDTH)):
                raise DatasetRecordingError(
                    f"decoded {name} frame has unexpected shape {tuple(image.shape)}"
                )


def validate_finalized_dataset(dataset: Dataset) -> None:
    """Validate the local schema and nonempty counters before Hub upload."""
    if not _features_match(dataset.meta.features, dataset_features()):
        raise DatasetRecordingError(
            "finalized dataset schema does not match the MolmoAct2 Bimanual YAM contract"
        )
    if dataset.meta.fps != DATASET_FPS:
        raise DatasetRecordingError(
            f"finalized dataset is {dataset.meta.fps} Hz; {DATASET_FPS} Hz is required"
        )
    if dataset.meta.total_episodes <= 0 or dataset.meta.total_frames <= 0:
        raise DatasetRecordingError("finalized dataset has no saved frames to upload")


def _check_free_disk(config: DatasetConfig, dataset: Dataset) -> None:
    root_value = getattr(dataset, "root", None) or config.root or Path.cwd()
    root = Path(root_value).expanduser()
    probe = root
    while not probe.exists() and probe != probe.parent:
        probe = probe.parent
    try:
        free_bytes = shutil.disk_usage(probe).free
    except OSError as exc:
        raise DatasetRecordingError(
            f"cannot check free space for dataset root {root}: {exc}"
        ) from exc
    required = int(config.min_free_disk_gb * 1024**3)
    if free_bytes < required:
        raise DatasetRecordingError(
            f"dataset root {root} has {free_bytes / 1024**3:.1f} GiB free; "
            f"at least {config.min_free_disk_gb:.1f} GiB is required"
        )


def _default_camera_factory(config: DatasetConfig) -> dict[str, Camera]:
    try:
        from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
    except ImportError as exc:
        raise DatasetRecordingError(
            "RealSense recording dependencies are unavailable; install the dataset extra"
        ) from exc

    cameras: dict[str, Camera] = {}
    try:
        for name in CAMERA_NAMES:
            capture_height = TOP_CAMERA_CAPTURE_HEIGHT if name == "top" else CAMERA_HEIGHT
            camera = RealSenseCamera(
                RealSenseCameraConfig(
                    serial_number_or_name=config.camera_serials[name],
                    fps=DATASET_FPS,
                    width=CAMERA_WIDTH,
                    height=capture_height,
                    use_rgb=True,
                    use_depth=False,
                    warmup_s=2,
                )
            )
            camera.connect()
            cameras[name] = camera
    except Exception as exc:
        for camera in reversed(tuple(cameras.values())):
            try:
                camera.disconnect()
            except Exception:
                pass
        if isinstance(exc, DatasetRecordingError):
            raise
        raise DatasetRecordingError(f"failed to initialize RealSense cameras: {exc}") from exc
    return cameras


@dataclass(frozen=True)
class _FrameOperation:
    frame: dict[str, Any]


@dataclass(frozen=True)
class _FinishOperation:
    commit: bool
    frame_count: int
    reason: str


_STOP = object()


class DatasetRecorder:
    """Capture frames in the control thread and write them on a bounded worker."""

    def __init__(
        self,
        config: DatasetConfig,
        dataset: Dataset,
        cameras: dict[str, Camera],
        upload_validator: Callable[[DatasetConfig, Dataset], None] | None = None,
    ) -> None:
        self.config = config
        self.dataset = dataset
        self.cameras = cameras
        self._upload_validator = upload_validator
        self._queue: Queue[object] = Queue(maxsize=config.queue_size)
        self._error_lock = Lock()
        self._worker_error: BaseException | None = None
        self._client_error: BaseException | None = None
        self._active = False
        self._episode_frames = 0
        self._saved_episodes = 0
        self._closed = False
        self._thread = Thread(target=self._write_loop, name="umi-dataset-writer", daemon=True)
        self._thread.start()

    @property
    def active(self) -> bool:
        return self._active

    def _set_worker_error(self, exc: BaseException) -> None:
        with self._error_lock:
            if self._worker_error is None:
                self._worker_error = exc

    def _current_error(self) -> BaseException | None:
        with self._error_lock:
            return self._client_error or self._worker_error

    def _check_healthy(self) -> None:
        if self._closed:
            raise DatasetRecordingError("dataset recorder is closed")
        if error := self._current_error():
            raise DatasetRecordingError(f"dataset writer failed: {error}") from error

    def _put_nowait(self, operation: object) -> None:
        self._check_healthy()
        try:
            self._queue.put_nowait(operation)
        except Full as exc:
            error = DatasetRecordingError(
                "dataset writer queue is full; recording cannot keep up with control"
            )
            with self._error_lock:
                self._client_error = error
            raise error from exc

    def start_episode(self) -> None:
        self._check_healthy()
        if self._active:
            raise DatasetRecordingError("dataset episode is already active")
        _check_free_disk(self.config, self.dataset)
        self._active = True
        self._episode_frames = 0
        print(f"DATASET episode started task={self.config.task!r}", flush=True)

    def record_frame(
        self,
        *,
        measured_joints: Mapping[str, np.ndarray],
        measured_grippers: Mapping[str, float],
        commanded_joints: Mapping[str, np.ndarray],
        commanded_grippers: Mapping[str, float],
    ) -> None:
        frame = self.prepare_frame(
            measured_joints=measured_joints,
            measured_grippers=measured_grippers,
            commanded_joints=commanded_joints,
            commanded_grippers=commanded_grippers,
        )
        self.commit_frame(frame)

    def prepare_frame(
        self,
        *,
        measured_joints: Mapping[str, np.ndarray],
        measured_grippers: Mapping[str, float],
        commanded_joints: Mapping[str, np.ndarray],
        commanded_grippers: Mapping[str, float],
    ) -> dict[str, Any]:
        """Copy one pre-command observation without mutating the dataset writer."""
        self._check_healthy()
        if not self._active:
            raise DatasetRecordingError("cannot record a frame without an active episode")
        try:
            images = {
                name: normalize_camera_image(
                    name,
                    camera.read_latest(max_age_ms=self.config.camera_max_age_ms),
                )
                for name, camera in self.cameras.items()
            }
        except Exception as exc:
            raise DatasetFrameError(f"camera capture failed: {exc}") from exc
        return build_frame(
            images=images,
            measured_joints=measured_joints,
            measured_grippers=measured_grippers,
            commanded_joints=commanded_joints,
            commanded_grippers=commanded_grippers,
            task=(self.config.task or "").strip(),
        )

    def commit_frame(self, frame: dict[str, Any]) -> None:
        """Queue a prepared frame after both arm command calls have succeeded."""
        self._check_healthy()
        if not self._active:
            raise DatasetRecordingError("cannot commit a frame without an active episode")
        self._put_nowait(_FrameOperation(frame))
        self._episode_frames += 1

    def finish_episode(self, *, commit: bool, reason: str) -> None:
        if not self._active:
            return
        self._check_healthy()
        frame_count = self._episode_frames
        commit = commit and frame_count >= self.config.min_episode_frames
        if frame_count < self.config.min_episode_frames:
            reason = (
                f"{reason}; only {frame_count} frames, minimum is {self.config.min_episode_frames}"
            )
        self._put_nowait(_FinishOperation(commit, frame_count, reason))
        self._active = False
        self._episode_frames = 0
        disposition = "save queued" if commit else "discard queued"
        print(
            f"DATASET episode {disposition} frames={frame_count} reason={reason}",
            flush=True,
        )

    def _write_loop(self) -> None:
        try:
            while True:
                operation = self._queue.get()
                if operation is _STOP:
                    return
                if isinstance(operation, _FrameOperation):
                    self.dataset.add_frame(operation.frame)
                elif isinstance(operation, _FinishOperation):
                    if operation.commit:
                        self.dataset.save_episode()
                        self._saved_episodes += 1
                        print(f"DATASET episode saved frames={operation.frame_count}", flush=True)
                    else:
                        self.dataset.clear_episode_buffer()
                        print(
                            f"DATASET episode discarded frames={operation.frame_count} "
                            f"reason={operation.reason}",
                            flush=True,
                        )
                else:
                    raise RuntimeError(f"unknown dataset operation {operation!r}")
        except BaseException as exc:
            self._set_worker_error(exc)

    def _put_for_close(self, operation: object, deadline: float) -> bool:
        while self._thread.is_alive() and time.monotonic() < deadline:
            try:
                timeout = min(0.1, max(0.0, deadline - time.monotonic()))
                self._queue.put(operation, timeout=timeout)
                return True
            except Full:
                continue
        return False

    def close(self, *, upload: bool = True) -> None:
        if self._closed:
            if error := self._current_error():
                raise DatasetRecordingError(f"dataset recorder failed: {error}") from error
            return

        close_errors: list[BaseException] = []
        deadline = time.monotonic() + self.config.shutdown_timeout_s
        if self._active:
            queued = self._put_for_close(
                _FinishOperation(False, self._episode_frames, "controller stopped mid-episode"),
                deadline,
            )
            if not queued and self._current_error() is None:
                close_errors.append(RuntimeError("dataset writer stopped before episode discard"))
            self._active = False
            self._episode_frames = 0
        if self._thread.is_alive():
            self._put_for_close(_STOP, deadline)
            self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        writer_still_running = self._thread.is_alive()
        if writer_still_running:
            close_errors.append(
                TimeoutError(
                    f"dataset writer did not stop within {self.config.shutdown_timeout_s:.1f}s"
                )
            )
        if error := self._current_error():
            close_errors.append(error)
            if not writer_still_running:
                try:
                    self.dataset.clear_episode_buffer()
                except BaseException as exc:
                    close_errors.append(exc)

        for camera in reversed(tuple(self.cameras.values())):
            try:
                camera.disconnect()
            except BaseException as exc:
                close_errors.append(exc)

        if not writer_still_running:
            try:
                self.dataset.finalize()
            except BaseException as exc:
                close_errors.append(exc)

        if not close_errors and self._saved_episodes > 0 and self._upload_validator is not None:
            try:
                self._upload_validator(self.config, self.dataset)
            except BaseException as exc:
                close_errors.append(exc)

        if upload and self.config.push_to_hub and not close_errors and self._saved_episodes > 0:
            try:
                self.dataset.push_to_hub(private=self.config.private)
            except BaseException as exc:
                close_errors.append(exc)
        elif upload and self.config.push_to_hub and not close_errors:
            print("DATASET upload skipped because no new episodes were saved", flush=True)
        elif not upload and self.config.push_to_hub and not close_errors:
            print(
                "DATASET upload deferred because controller shutdown had an error; "
                "use umi-dataset-upload after checking the robot",
                flush=True,
            )

        self._closed = True
        if close_errors:
            detail = "; ".join(str(error) for error in close_errors)
            error = DatasetRecordingError(f"dataset cleanup/upload failed: {detail}")
            with self._error_lock:
                self._client_error = error
            raise error from close_errors[0]


def create_dataset_recorder(
    config: DatasetConfig,
    control_hz: float,
    *,
    dataset_factory: DatasetFactory = _default_dataset_factory,
    camera_factory: CameraFactory = _default_camera_factory,
) -> DatasetRecorder | None:
    config.validate(control_hz)
    if not config.enabled:
        return None

    try:
        cameras = camera_factory(config)
    except DatasetRecordingError:
        raise
    except Exception as exc:
        raise DatasetRecordingError(f"failed to initialize dataset cameras: {exc}") from exc
    if set(cameras) != set(CAMERA_NAMES):
        for camera in cameras.values():
            try:
                camera.disconnect()
            except Exception:
                pass
        raise DatasetRecordingError("camera factory must return top, left, and right")
    try:
        dataset = dataset_factory(config, dataset_features())
    except Exception as exc:
        for camera in reversed(tuple(cameras.values())):
            try:
                camera.disconnect()
            except Exception:
                pass
        if isinstance(exc, DatasetRecordingError):
            raise
        raise DatasetRecordingError(f"failed to initialize LeRobot dataset: {exc}") from exc
    try:
        _check_free_disk(config, dataset)
        upload_validator = (
            _default_upload_validator if dataset_factory is _default_dataset_factory else None
        )
        return DatasetRecorder(config, dataset, cameras, upload_validator)
    except Exception as exc:
        for camera in reversed(tuple(cameras.values())):
            try:
                camera.disconnect()
            except Exception:
                pass
        try:
            dataset.finalize()
        except Exception:
            pass
        if isinstance(exc, DatasetRecordingError):
            raise
        raise DatasetRecordingError(f"failed to start dataset recorder: {exc}") from exc


def prepare_dataset_step(
    recorder: DatasetRecorder,
    *,
    was_armed: bool,
    is_armed: bool,
    invalid_frame_reason: str | None,
    measured_joints: Mapping[str, np.ndarray],
    measured_grippers: Mapping[str, float],
    commanded_joints: Mapping[str, np.ndarray],
    commanded_grippers: Mapping[str, float],
) -> dict[str, Any] | None:
    """Capture a pre-command observation for one motor-loop result."""
    if not was_armed and is_armed:
        recorder.start_episode()
    if is_armed and recorder.active:
        if invalid_frame_reason is not None:
            raise DatasetFrameError(invalid_frame_reason)
        else:
            return recorder.prepare_frame(
                measured_joints=measured_joints,
                measured_grippers=measured_grippers,
                commanded_joints=commanded_joints,
                commanded_grippers=commanded_grippers,
            )
    return None


def commit_dataset_step(
    recorder: DatasetRecorder,
    prepared_frame: dict[str, Any] | None,
    *,
    was_armed: bool,
    is_armed: bool,
    disarm_reason: str,
) -> None:
    """Commit after successful bimanual dispatch, then close any disarmed episode."""
    if prepared_frame is not None:
        recorder.commit_frame(prepared_frame)
    if was_armed and not is_armed:
        recorder.finish_episode(
            commit=disarm_reason == "enable toggled off",
            reason=disarm_reason,
        )


def update_dataset_episode(
    recorder: DatasetRecorder,
    *,
    was_armed: bool,
    is_armed: bool,
    disarm_reason: str,
    invalid_frame_reason: str | None,
    measured_joints: Mapping[str, np.ndarray],
    measured_grippers: Mapping[str, float],
    commanded_joints: Mapping[str, np.ndarray],
    commanded_grippers: Mapping[str, float],
) -> None:
    """Convenience wrapper for tests and non-hardware callers."""
    frame = prepare_dataset_step(
        recorder,
        was_armed=was_armed,
        is_armed=is_armed,
        invalid_frame_reason=invalid_frame_reason,
        measured_joints=measured_joints,
        measured_grippers=measured_grippers,
        commanded_joints=commanded_joints,
        commanded_grippers=commanded_grippers,
    )
    commit_dataset_step(
        recorder,
        frame,
        was_armed=was_armed,
        is_armed=is_armed,
        disarm_reason=disarm_reason,
    )

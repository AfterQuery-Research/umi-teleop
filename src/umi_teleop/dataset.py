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
from types import SimpleNamespace
from typing import Any, Literal, Protocol

import numpy as np

from umi_teleop.controller import ARM_NAMES

CAMERA_NAMES = ("top", "left", "right")
CAMERA_HEIGHT = 270
CAMERA_WIDTH = 480
CAMERA_SHAPE = (CAMERA_HEIGHT, CAMERA_WIDTH, 3)
TOP_CAMERA_CAPTURE_WIDTH = 640
TOP_CAMERA_CAPTURE_HEIGHT = 480
TOP_CAMERA_CROP_HEIGHT = 360
ULTRAWIDE_CAPTURE_HEIGHT = 480
ULTRAWIDE_CAPTURE_WIDTH = 640
ULTRAWIDE_CAMERA_HEIGHT = 312
ULTRAWIDE_CAMERA_WIDTH = 416
ULTRAWIDE_CAMERA_SHAPE = (ULTRAWIDE_CAMERA_HEIGHT, ULTRAWIDE_CAMERA_WIDTH, 3)
ULTRAWIDE_REQUESTED_FPS = 60
DATASET_FPS = 30
SideCameraKind = Literal["realsense", "ultrawide"]
YAM_SCALAR_NAMES = (
    *(f"left_joint_{index}.pos" for index in range(6)),
    "left_gripper.pos",
    *(f"right_joint_{index}.pos" for index in range(6)),
    "right_gripper.pos",
)


@dataclass(frozen=True)
class DatasetConfig:
    """Opt-in LeRobot dataset settings for ``umi-yam``."""

    record: bool = False
    """Enable dataset recording; a repo ID is still required."""
    repo_id: str | None = None
    root: Path | None = None
    task: str | None = None
    top_camera_serial: str | None = None
    left_camera_serial: str | None = None
    right_camera_serial: str | None = None
    side_camera_kind: SideCameraKind = "realsense"
    left_camera_path: Path | None = None
    right_camera_path: Path | None = None
    resume: bool = False
    push_to_hub: bool = False
    private: bool = True
    recording_fps: int = DATASET_FPS
    post_episode_action: Literal["hold", "neutral"] = "hold"
    min_episode_frames: int = 15
    queue_size: int = 300
    camera_max_age_ms: int = 250
    max_control_lag_s: float = 0.10
    control_lag_grace_s: float = 0.50
    min_free_disk_gb: float = 5.0
    image_writer_threads_per_camera: int = 2
    encoder_threads: int = 2
    streaming_encoding: bool = False
    encoder_queue_size: int = 120
    encoder_gop_size: int = DATASET_FPS
    encoder_preset: str | None = None
    video_codec: Literal["h264", "libsvtav1"] = "h264"
    shutdown_timeout_s: float = 60.0

    @property
    def enabled(self) -> bool:
        return self.record or self.repo_id is not None

    @property
    def camera_serials(self) -> dict[str, str]:
        values = {
            "top": self.top_camera_serial,
            "left": self.left_camera_serial,
            "right": self.right_camera_serial,
        }
        return {name: value.strip() for name, value in values.items() if value is not None}

    @property
    def camera_paths(self) -> dict[str, Path]:
        values = {"left": self.left_camera_path, "right": self.right_camera_path}
        return {
            name: Path(value).expanduser() for name, value in values.items() if value is not None
        }

    @property
    def camera_shapes(self) -> dict[str, tuple[int, int, int]]:
        return dataset_camera_shapes(self.side_camera_kind)

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
        if self.recording_fps != DATASET_FPS:
            raise DatasetRecordingError(
                f"dataset recording frequency must be {DATASET_FPS} Hz for MolmoAct2"
            )
        if control_hz < self.recording_fps:
            raise DatasetRecordingError(
                "YAM control frequency must be at least the dataset recording frequency"
            )
        if self.post_episode_action not in ("hold", "neutral"):
            raise DatasetRecordingError("post-episode action must be hold or neutral")
        if self.side_camera_kind not in ("realsense", "ultrawide"):
            raise DatasetRecordingError(
                "dataset side camera kind must be realsense or ultrawide, "
                f"got {self.side_camera_kind!r}"
            )
        serials = self.camera_serials
        if not serials.get("top"):
            raise DatasetRecordingError("dataset recording requires a camera serial for top")
        if self.side_camera_kind == "realsense":
            missing = [name for name in ("left", "right") if not serials.get(name)]
            if missing:
                raise DatasetRecordingError(
                    "dataset recording requires camera serials for " + ", ".join(missing)
                )
            if len({serials[name] for name in CAMERA_NAMES}) != len(CAMERA_NAMES):
                raise DatasetRecordingError("top, left, and right camera serials must be distinct")
        else:
            paths = self.camera_paths
            missing = [name for name in ("left", "right") if name not in paths]
            if missing:
                raise DatasetRecordingError(
                    "ultrawide recording requires camera paths for " + ", ".join(missing)
                )
            if paths["left"] == paths["right"]:
                raise DatasetRecordingError(
                    "left and right ultrawide camera paths must be distinct"
                )
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
            ("encoder queue size", self.encoder_queue_size),
            ("encoder GOP size", self.encoder_gop_size),
        ):
            if value <= 0:
                raise DatasetRecordingError(f"dataset {name} must be positive")
        if self.max_control_lag_s <= 0.0:
            raise DatasetRecordingError("dataset max control lag must be positive")
        if self.control_lag_grace_s <= 0.0:
            raise DatasetRecordingError("dataset control lag grace must be positive")
        if self.min_free_disk_gb <= 0.0:
            raise DatasetRecordingError("dataset minimum free disk space must be positive")
        if self.shutdown_timeout_s <= 0.0:
            raise DatasetRecordingError("dataset shutdown timeout must be positive")


class DatasetRecordingError(RuntimeError):
    pass


class DatasetFrameError(DatasetRecordingError):
    """A recoverable frame/camera error that invalidates only the active episode."""


@dataclass(frozen=True)
class ControlLagDecision:
    """Result of one cumulative control-lag observation."""

    over_limit: bool
    fault: bool
    over_limit_for_s: float


class ControlLagMonitor:
    """Allow a transient scheduling backlog to recover, but reject sustained overload."""

    def __init__(self, limit_s: float, grace_s: float):
        if limit_s <= 0.0 or grace_s <= 0.0:
            raise ValueError("control lag limit and grace must be positive")
        self.limit_s = float(limit_s)
        self.grace_s = float(grace_s)
        self._over_limit_since: float | None = None

    def reset(self) -> None:
        self._over_limit_since = None

    def observe(
        self,
        lag_s: float,
        *,
        cycle_overrun_s: float,
        now: float | None = None,
    ) -> ControlLagDecision:
        timestamp = time.monotonic() if now is None else float(now)
        lag = max(0.0, float(lag_s))
        cycle_overrun = max(0.0, float(cycle_overrun_s))
        if lag <= self.limit_s:
            self.reset()
            return ControlLagDecision(False, False, 0.0)
        if self._over_limit_since is None:
            self._over_limit_since = timestamp
        duration = max(0.0, timestamp - self._over_limit_since)
        severe_single_cycle = cycle_overrun > self.limit_s + self.grace_s
        return ControlLagDecision(True, severe_single_cycle or duration >= self.grace_s, duration)


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


def dataset_camera_shapes(
    side_camera_kind: SideCameraKind = "realsense",
) -> dict[str, tuple[int, int, int]]:
    if side_camera_kind == "realsense":
        side_shape = CAMERA_SHAPE
    elif side_camera_kind == "ultrawide":
        side_shape = ULTRAWIDE_CAMERA_SHAPE
    else:
        raise ValueError(f"unsupported side camera kind {side_camera_kind!r}")
    return {"top": CAMERA_SHAPE, "left": side_shape, "right": side_shape}


def dataset_features(
    side_camera_kind: SideCameraKind = "realsense",
) -> dict[str, dict[str, Any]]:
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
    for name, shape in dataset_camera_shapes(side_camera_kind).items():
        features[f"observation.images.{name}"] = {
            "dtype": "video",
            "shape": shape,
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
    camera_shapes: Mapping[str, tuple[int, int, int]] | None = None,
) -> dict[str, Any]:
    if set(images) != set(CAMERA_NAMES):
        raise DatasetFrameError("camera frames must contain top, left, and right")
    frame: dict[str, Any] = {
        "observation.state": yam_vector(measured_joints, measured_grippers, label="measured state"),
        "action": yam_vector(commanded_joints, commanded_grippers, label="commanded action"),
        "task": task,
    }
    expected_shapes = camera_shapes or dataset_camera_shapes()
    for name in CAMERA_NAMES:
        image = np.asarray(images[name])
        expected_shape = expected_shapes[name]
        if image.shape != expected_shape:
            raise DatasetFrameError(
                f"{name} camera frame shape {image.shape} does not match {expected_shape}"
            )
        if image.dtype != np.uint8:
            raise DatasetFrameError(f"{name} camera frame must be RGB uint8")
        frame[f"observation.images.{name}"] = np.array(image, copy=True, order="C")
    return frame


def _validated_camera_image(
    name: str,
    image: np.ndarray,
    expected_shape: tuple[int, int, int],
) -> np.ndarray:
    """Validate that a camera frame can be converted to its stored shape without copying it."""
    image = np.asarray(image)
    accepted_shapes = {expected_shape}
    if name == "top" and expected_shape == CAMERA_SHAPE:
        accepted_shapes.add((TOP_CAMERA_CAPTURE_HEIGHT, TOP_CAMERA_CAPTURE_WIDTH, 3))
    elif name in ("left", "right") and expected_shape == ULTRAWIDE_CAMERA_SHAPE:
        accepted_shapes.add((ULTRAWIDE_CAPTURE_HEIGHT, ULTRAWIDE_CAPTURE_WIDTH, 3))
    if image.shape not in accepted_shapes:
        raise DatasetFrameError(
            f"{name} camera frame shape {image.shape} cannot be converted to {expected_shape}"
        )
    if image.dtype != np.uint8:
        raise DatasetFrameError(f"{name} camera frame must be RGB uint8")
    return image


def _camera_image_view(
    name: str,
    image: np.ndarray,
    camera_shapes: Mapping[str, tuple[int, int, int]] | None = None,
) -> np.ndarray:
    """Validate and normalize a camera image to its stored dataset resolution."""
    expected_shape = (camera_shapes or dataset_camera_shapes())[name]
    image = _validated_camera_image(name, image, expected_shape)
    if name == "top" and image.shape == (
        TOP_CAMERA_CAPTURE_HEIGHT,
        TOP_CAMERA_CAPTURE_WIDTH,
        3,
    ):
        crop_start = (TOP_CAMERA_CAPTURE_HEIGHT - TOP_CAMERA_CROP_HEIGHT) // 2
        image = image[crop_start : crop_start + TOP_CAMERA_CROP_HEIGHT]
        try:
            import cv2
        except ImportError as exc:
            raise DatasetFrameError(
                "top camera downsampling requires OpenCV from the dataset dependencies"
            ) from exc
        image = cv2.resize(
            image,
            (CAMERA_WIDTH, CAMERA_HEIGHT),
            interpolation=cv2.INTER_AREA,
        )
    elif (
        name in ("left", "right")
        and image.shape
        == (
            ULTRAWIDE_CAPTURE_HEIGHT,
            ULTRAWIDE_CAPTURE_WIDTH,
            3,
        )
        and expected_shape == ULTRAWIDE_CAMERA_SHAPE
    ):
        try:
            import cv2
        except ImportError as exc:
            raise DatasetFrameError(
                "ultrawide camera downsampling requires OpenCV from the dataset dependencies"
            ) from exc
        image = cv2.resize(
            image,
            (ULTRAWIDE_CAMERA_WIDTH, ULTRAWIDE_CAMERA_HEIGHT),
            interpolation=cv2.INTER_AREA,
        )
    assert image.shape == expected_shape
    return image


def normalize_camera_image(
    name: str,
    image: np.ndarray,
    side_camera_kind: SideCameraKind = "realsense",
) -> np.ndarray:
    """Return one RGB camera image in the selected LeRobot dataset shape."""
    return np.array(
        _camera_image_view(name, image, dataset_camera_shapes(side_camera_kind)),
        copy=True,
        order="C",
    )


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


def _effective_encoder_preset(config: DatasetConfig) -> str | None:
    """Prefer low-latency H.264 encoding while preserving explicit overrides."""
    if config.encoder_preset is not None:
        return config.encoder_preset
    if config.streaming_encoding and config.video_codec == "h264":
        return "ultrafast"
    return None


def _default_dataset_factory(config: DatasetConfig, features: dict[str, dict[str, Any]]) -> Dataset:
    try:
        from lerobot.configs import RGBEncoderConfig
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        from umi_teleop.streaming_encoder import install_streaming_encoder_optimization
    except ImportError as exc:
        raise DatasetRecordingError(
            "LeRobot recording dependencies are unavailable; install the dataset extra"
        ) from exc

    if config.streaming_encoding:
        install_streaming_encoder_optimization()

    common = {
        "root": config.root,
        "image_writer_processes": 0,
        "image_writer_threads": (
            0
            if config.streaming_encoding
            else config.image_writer_threads_per_camera * len(CAMERA_NAMES)
        ),
        "batch_encoding_size": 1,
        "encoder_threads": config.encoder_threads,
        "streaming_encoding": config.streaming_encoding,
        "encoder_queue_maxsize": config.encoder_queue_size,
        "rgb_encoder": RGBEncoderConfig(
            vcodec=config.video_codec,
            pix_fmt="yuv420p",
            g=config.encoder_gop_size,
            preset=_effective_encoder_preset(config),
        ),
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
    expected_features = dataset_features(config.side_camera_kind)
    validate_finalized_dataset(reopened, expected_features=expected_features)
    from umi_teleop.streaming_encoder import _suppress_av_conversion_warnings

    with _suppress_av_conversion_warnings():
        for frame_index in sorted({0, reopened.meta.total_frames - 1}):
            sample = reopened[frame_index]
            for name in CAMERA_NAMES:
                image = sample[f"observation.images.{name}"]
                expected_shape = config.camera_shapes[name]
                channel_first_shape = (expected_shape[2], expected_shape[0], expected_shape[1])
                if tuple(image.shape) not in (expected_shape, channel_first_shape):
                    raise DatasetRecordingError(
                        f"decoded {name} frame has unexpected shape {tuple(image.shape)}"
                    )


def validate_finalized_dataset(
    dataset: Dataset,
    *,
    expected_features: Mapping[str, Mapping[str, Any]] | None = None,
) -> None:
    """Validate the local schema, counters, and encoded media before Hub upload."""
    allowed_features = (
        (expected_features,)
        if expected_features is not None
        else (dataset_features("realsense"), dataset_features("ultrawide"))
    )
    if not any(_features_match(dataset.meta.features, expected) for expected in allowed_features):
        raise DatasetRecordingError(
            "finalized dataset schema does not match the MolmoAct2 Bimanual YAM contract"
        )
    if dataset.meta.fps != DATASET_FPS:
        raise DatasetRecordingError(
            f"finalized dataset is {dataset.meta.fps} Hz; {DATASET_FPS} Hz is required"
        )
    if dataset.meta.total_episodes <= 0 or dataset.meta.total_frames <= 0:
        raise DatasetRecordingError("finalized dataset has no saved frames to upload")
    _validate_video_integrity(dataset)


def _video_frame_count(video_path: Path) -> int:
    """Read a video container's frame count without decoding the full stream."""
    try:
        import av
    except ImportError as exc:
        raise DatasetRecordingError(
            "video integrity validation requires PyAV from the dataset dependencies"
        ) from exc
    try:
        with av.open(str(video_path)) as container:
            if not container.streams.video:
                raise DatasetRecordingError(f"encoded video has no video stream: {video_path}")
            stream = container.streams.video[0]
            if stream.frames > 0:
                return int(stream.frames)
            return sum(1 for packet in container.demux(stream) if packet.dts is not None)
    except DatasetRecordingError:
        raise
    except Exception as exc:
        raise DatasetRecordingError(f"cannot inspect encoded video {video_path}: {exc}") from exc


def _validate_video_integrity(dataset: Dataset) -> None:
    """Require every finalized camera video to agree with episode metadata."""
    root_value = getattr(dataset, "root", None)
    if root_value is None:
        # Lightweight fake datasets used by unit tests do not contain encoded media.
        return
    root = Path(root_value)
    meta = dataset.meta
    episodes = getattr(meta, "episodes", None)
    if episodes is None:
        raise DatasetRecordingError("finalized dataset is missing episode metadata")
    video_keys = [key for key, feature in meta.features.items() if feature.get("dtype") == "video"]
    video_files: dict[tuple[str, Path], dict[str, Any]] = {}
    expected_from = 0
    for episode_index, episode in enumerate(episodes):
        try:
            length = int(episode["length"])
            dataset_from = int(episode["dataset_from_index"])
            dataset_to = int(episode["dataset_to_index"])
        except (KeyError, TypeError, ValueError) as exc:
            raise DatasetRecordingError(
                f"episode {episode_index} has invalid tabular metadata"
            ) from exc
        if dataset_from != expected_from or dataset_to != dataset_from + length:
            raise DatasetRecordingError(
                f"episode {episode_index} row range [{dataset_from}, {dataset_to}) "
                f"does not match length {length} after row {expected_from}"
            )
        for video_key in video_keys:
            try:
                relative_path = meta.get_video_file_path(episode_index, video_key)
            except Exception as exc:
                raise DatasetRecordingError(
                    f"episode {episode_index} is missing video metadata for {video_key}"
                ) from exc
            video_path = root / relative_path
            if not video_path.is_file():
                raise DatasetRecordingError(
                    f"episode {episode_index} video is missing for {video_key}: {video_path}"
                )
            entry = video_files.setdefault(
                (video_key, video_path),
                {"expected_frames": 0, "episodes": []},
            )
            entry["expected_frames"] += length
            entry["episodes"].append(episode_index)
        expected_from = dataset_to
    if len(episodes) != int(meta.total_episodes) or expected_from != int(meta.total_frames):
        raise DatasetRecordingError(
            "episode metadata totals do not match finalized dataset counters: "
            f"episodes={len(episodes)}/{meta.total_episodes} "
            f"frames={expected_from}/{meta.total_frames}"
        )
    for (video_key, video_path), entry in video_files.items():
        frame_count = _video_frame_count(video_path)
        expected_frames = int(entry["expected_frames"])
        episode_indices = entry["episodes"]
        if frame_count != expected_frames:
            if len(episode_indices) == 1:
                label = f"episode {episode_indices[0]} {video_key}"
            else:
                joined = ",".join(str(index) for index in episode_indices)
                label = f"{video_key} shared video for episodes [{joined}]"
            raise DatasetRecordingError(
                f"{label} has {frame_count} encoded frames; expected {expected_frames}"
            )


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


@dataclass(frozen=True)
class _CameraSpec:
    name: str
    backend: Literal["realsense", "opencv"]
    source: str | Path
    width: int
    height: int
    requested_fps: int
    fourcc: str | None = None


def _camera_specs(config: DatasetConfig) -> tuple[_CameraSpec, ...]:
    specs = [
        _CameraSpec(
            "top",
            "realsense",
            config.camera_serials["top"],
            TOP_CAMERA_CAPTURE_WIDTH,
            TOP_CAMERA_CAPTURE_HEIGHT,
            DATASET_FPS,
        )
    ]
    if config.side_camera_kind == "realsense":
        specs.extend(
            _CameraSpec(
                name,
                "realsense",
                config.camera_serials[name],
                CAMERA_WIDTH,
                CAMERA_HEIGHT,
                DATASET_FPS,
            )
            for name in ("left", "right")
        )
    else:
        specs.extend(
            _CameraSpec(
                name,
                "opencv",
                config.camera_paths[name],
                ULTRAWIDE_CAPTURE_WIDTH,
                ULTRAWIDE_CAPTURE_HEIGHT,
                ULTRAWIDE_REQUESTED_FPS,
                "MJPG",
            )
            for name in ("left", "right")
        )
    return tuple(specs)


def _default_camera_factory(config: DatasetConfig) -> dict[str, Camera]:
    try:
        from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
    except ImportError as exc:
        raise DatasetRecordingError(
            "RealSense recording dependencies are unavailable; install the dataset extra"
        ) from exc
    if config.side_camera_kind == "ultrawide":
        try:
            from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
        except ImportError as exc:
            raise DatasetRecordingError(
                "OpenCV recording dependencies are unavailable; install the dataset extra"
            ) from exc

    cameras: dict[str, Camera] = {}
    try:
        for spec in _camera_specs(config):
            if spec.backend == "realsense":
                camera = RealSenseCamera(
                    RealSenseCameraConfig(
                        serial_number_or_name=str(spec.source),
                        fps=spec.requested_fps,
                        width=spec.width,
                        height=spec.height,
                        use_rgb=True,
                        use_depth=False,
                        warmup_s=2,
                    )
                )
            else:
                camera = OpenCVCamera(
                    OpenCVCameraConfig(
                        index_or_path=Path(spec.source),
                        fps=spec.requested_fps,
                        width=spec.width,
                        height=spec.height,
                        warmup_s=2,
                        fourcc=spec.fourcc,
                    )
                )
            camera.connect()
            cameras[spec.name] = camera
    except Exception as exc:
        for camera in reversed(tuple(cameras.values())):
            try:
                camera.disconnect()
            except Exception:
                pass
        if isinstance(exc, DatasetRecordingError):
            raise
        raise DatasetRecordingError(
            f"failed to initialize {config.side_camera_kind} dataset cameras: {exc}"
        ) from exc
    return cameras


def _validate_huggingface_auth() -> None:
    try:
        from huggingface_hub import HfApi
    except ImportError as exc:
        raise DatasetRecordingError(
            "Hugging Face upload requires huggingface_hub from the dataset dependencies"
        ) from exc
    try:
        identity = HfApi().whoami()
    except Exception as exc:
        raise DatasetRecordingError(
            "Hugging Face authentication failed; run `hf auth login` or set HF_TOKEN"
        ) from exc
    if not isinstance(identity, dict) or not identity.get("name"):
        raise DatasetRecordingError("Hugging Face authentication returned no user identity")


def preflight_dataset(
    config: DatasetConfig,
    control_hz: float,
    *,
    camera_factory: CameraFactory = _default_camera_factory,
    auth_validator: Callable[[], None] = _validate_huggingface_auth,
) -> None:
    """Validate dataset storage, observation cameras, and optional Hub authentication."""
    config.validate(control_hz)
    if not config.enabled:
        return
    _check_free_disk(config, SimpleNamespace(root=config.root))
    cameras: dict[str, Camera] = {}
    try:
        cameras = camera_factory(config)
        if set(cameras) != set(CAMERA_NAMES):
            raise DatasetRecordingError("camera factory must return top, left, and right")
        for name, camera in cameras.items():
            image = camera.read_latest(max_age_ms=config.camera_max_age_ms)
            _validated_camera_image(name, image, config.camera_shapes[name])
        if config.push_to_hub:
            auth_validator()
    except DatasetRecordingError:
        raise
    except Exception as exc:
        raise DatasetRecordingError(f"dataset preflight failed: {exc}") from exc
    finally:
        for camera in reversed(tuple(cameras.values())):
            try:
                camera.disconnect()
            except Exception:
                pass


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
        control_hz: float = DATASET_FPS,
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
        self._control_hz = float(control_hz)
        self._sample_accumulator = 0.0
        self._sample_started = False
        self._saved_episodes = 0
        self._closed = False
        self._thread = Thread(target=self._write_loop, name="umi-dataset-writer", daemon=True)
        self._thread.start()

    @property
    def active(self) -> bool:
        return self._active

    def queue_depths(self) -> dict[str, int]:
        """Return approximate queue depths for live throughput diagnostics."""
        depths = {"recorder": self._queue.qsize()}
        writer = getattr(self.dataset, "writer", None)
        image_writer = getattr(writer, "image_writer", None)
        image_queue = getattr(image_writer, "queue", None)
        if image_queue is not None:
            try:
                depths["png"] = image_queue.qsize()
            except (AttributeError, NotImplementedError):
                pass
        streaming_encoder = getattr(writer, "_streaming_encoder", None)
        frame_queues = getattr(streaming_encoder, "_frame_queues", {})
        for key, frame_queue in frame_queues.items():
            short_key = key.rsplit(".", maxsplit=1)[-1]
            depths[f"encoder_{short_key}"] = frame_queue.qsize()
        return depths

    def _streaming_encoder_drops(self) -> dict[str, int]:
        """Return positive per-camera drop counts reported by LeRobot's encoder."""
        writer = getattr(self.dataset, "writer", None)
        encoder = getattr(writer, "_streaming_encoder", None)
        raw_counts = getattr(encoder, "_dropped_frames", {})
        for _attempt in range(2):
            try:
                items = tuple(raw_counts.items())
                break
            except RuntimeError:
                continue
        else:
            raise DatasetRecordingError("streaming encoder drop counters changed while reading")
        return {str(key): int(value) for key, value in items if int(value) > 0}

    def _clear_streaming_encoder_drops(self) -> None:
        writer = getattr(self.dataset, "writer", None)
        encoder = getattr(writer, "_streaming_encoder", None)
        raw_counts = getattr(encoder, "_dropped_frames", None)
        if raw_counts is not None:
            raw_counts.clear()

    @staticmethod
    def _format_streaming_encoder_drops(drops: Mapping[str, int]) -> str:
        return ", ".join(f"{key}={value}" for key, value in sorted(drops.items()))

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

    def _check_recoverable_backlog(self) -> None:
        """Invalidate an episode before writer saturation can kill motor hold.

        Keep enough queue capacity for the caller to enqueue the current episode's
        discard operation. Very small queues are retained for deterministic overflow
        tests and are not useful as production recording configurations.
        """
        if self.config.queue_size < 4:
            return
        reserve = max(2, min(DATASET_FPS, self.config.queue_size // 10))
        limit = self.config.queue_size - reserve
        depth = self._queue.qsize()
        if depth >= limit:
            raise DatasetFrameError(
                f"dataset writer backlog reached {depth}/{self.config.queue_size} frames; "
                "recording cannot keep up, discarding episode before controller saturation"
            )

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
        self._sample_accumulator = 0.0
        self._sample_started = False
        print(f"DATASET episode started task={self.config.task!r}", flush=True)

    def should_sample(self, now_s: float | None = None) -> bool:
        """Return true at the fixed dataset rate while the control loop may run faster."""
        self._check_healthy()
        if not self._active:
            return False
        del now_s
        if not self._sample_started:
            self._sample_started = True
            return True
        self._sample_accumulator += self.config.recording_fps
        if self._sample_accumulator + 1e-9 < self._control_hz:
            return False
        self._sample_accumulator %= self._control_hz
        return True

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
        self._check_recoverable_backlog()
        drops = self._streaming_encoder_drops()
        if drops:
            raise DatasetFrameError(
                "streaming encoder dropped frames: " + self._format_streaming_encoder_drops(drops)
            )
        try:
            images = {}
            for name, camera in self.cameras.items():
                image = camera.read_latest(max_age_ms=self.config.camera_max_age_ms)
                # Validate convertibility now so a bad camera frame disarms immediately, but defer
                # the expensive resize to the dataset writer thread.  The private copy preserves
                # the exact pre-command observation even if the camera reuses its capture buffer.
                _validated_camera_image(name, image, self.config.camera_shapes[name])
                images[name] = np.array(image, copy=True, order="C")
        except Exception as exc:
            raise DatasetFrameError(f"camera capture failed: {exc}") from exc
        frame: dict[str, Any] = {
            "observation.state": yam_vector(
                measured_joints, measured_grippers, label="measured state"
            ),
            "action": yam_vector(commanded_joints, commanded_grippers, label="commanded action"),
            "task": (self.config.task or "").strip(),
        }
        for name, image in images.items():
            frame[f"observation.images.{name}"] = image
        return frame

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
        self._sample_accumulator = 0.0
        self._sample_started = False
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
                    frame = operation.frame
                    for name in CAMERA_NAMES:
                        key = f"observation.images.{name}"
                        frame[key] = np.array(
                            _camera_image_view(
                                name,
                                frame[key],
                                self.config.camera_shapes,
                            ),
                            copy=True,
                            order="C",
                        )
                    self.dataset.add_frame(frame)
                elif isinstance(operation, _FinishOperation):
                    drops = self._streaming_encoder_drops()
                    if operation.commit and not drops:
                        self.dataset.save_episode()
                        self._saved_episodes += 1
                        print(f"DATASET episode saved frames={operation.frame_count}", flush=True)
                    else:
                        self.dataset.clear_episode_buffer()
                        reason = operation.reason
                        if drops:
                            reason += "; streaming encoder dropped frames: "
                            reason += self._format_streaming_encoder_drops(drops)
                        print(
                            f"DATASET episode discarded frames={operation.frame_count} "
                            f"reason={reason}",
                            flush=True,
                        )
                    self._clear_streaming_encoder_drops()
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
                print(
                    "DATASET upload failed; the finalized local dataset was preserved. "
                    f"Retry with: umi-dataset-upload --repo-id {self.config.repo_id} "
                    f"--root {getattr(self.dataset, 'root', self.config.root)}",
                    flush=True,
                )
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
        dataset = dataset_factory(config, dataset_features(config.side_camera_kind))
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
        return DatasetRecorder(config, dataset, cameras, upload_validator, control_hz)
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
    if not was_armed and is_armed and not recorder.active:
        recorder.start_episode()
    if is_armed and recorder.active and recorder.should_sample():
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

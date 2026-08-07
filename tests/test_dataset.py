from __future__ import annotations

import time
from collections.abc import Callable
from pathlib import Path
from threading import Event
from typing import Any

import numpy as np
import pytest

from umi_teleop.dataset import (
    CAMERA_NAMES,
    CAMERA_SHAPE,
    TOP_CAMERA_CAPTURE_HEIGHT,
    TOP_CAMERA_CAPTURE_WIDTH,
    TOP_CAMERA_CROP_HEIGHT,
    ULTRAWIDE_CAMERA_SHAPE,
    ULTRAWIDE_CAPTURE_HEIGHT,
    ULTRAWIDE_CAPTURE_WIDTH,
    ULTRAWIDE_REQUESTED_FPS,
    YAM_SCALAR_NAMES,
    ControlLagMonitor,
    DatasetConfig,
    DatasetFrameError,
    DatasetRecordingError,
    _camera_specs,
    _effective_encoder_preset,
    build_frame,
    commit_dataset_step,
    create_dataset_recorder,
    dataset_camera_shapes,
    dataset_features,
    normalize_camera_image,
    prepare_dataset_step,
    validate_finalized_dataset,
    yam_vector,
)


class FakeCamera:
    def __init__(self, value: int = 0, shape: tuple[int, ...] = CAMERA_SHAPE) -> None:
        self.image = np.full(shape, value, dtype=np.uint8)
        self.disconnected = 0

    def read_latest(self, max_age_ms: int = 500) -> np.ndarray:
        assert max_age_ms > 0
        return self.image

    def disconnect(self) -> None:
        self.disconnected += 1


def test_control_lag_monitor_recovers_from_a_transient_backlog():
    monitor = ControlLagMonitor(limit_s=0.1, grace_s=0.5)

    entered = monitor.observe(0.15, cycle_overrun_s=0.08, now=10.0)
    recovering = monitor.observe(0.12, cycle_overrun_s=0.0, now=10.2)
    recovered = monitor.observe(0.08, cycle_overrun_s=0.0, now=10.3)

    assert entered.over_limit and not entered.fault
    assert recovering.over_limit and not recovering.fault
    assert not recovered.over_limit and not recovered.fault


def test_control_lag_monitor_rejects_sustained_or_severe_overload():
    monitor = ControlLagMonitor(limit_s=0.1, grace_s=0.5)
    monitor.observe(0.15, cycle_overrun_s=0.08, now=20.0)
    sustained = monitor.observe(0.14, cycle_overrun_s=0.08, now=20.5)
    assert sustained.fault

    monitor.reset()
    severe = monitor.observe(0.7, cycle_overrun_s=0.61, now=30.0)
    assert severe.fault


class FakeDataset:
    def __init__(self, fail_add: bool = False) -> None:
        self.meta = type("Meta", (), {"features": dataset_features()})()
        self.buffer: list[dict[str, Any]] = []
        self.episodes: list[list[dict[str, Any]]] = []
        self.clear_count = 0
        self.finalize_count = 0
        self.pushes: list[dict[str, Any]] = []
        self.fail_add = fail_add

    def add_frame(self, frame: dict[str, Any]) -> None:
        if self.fail_add:
            raise OSError("disk full")
        self.buffer.append(frame)

    def save_episode(self) -> None:
        self.episodes.append(self.buffer)
        self.buffer = []

    def clear_episode_buffer(self, delete_images: bool = True) -> None:
        assert delete_images
        self.buffer = []
        self.clear_count += 1

    def finalize(self) -> None:
        self.finalize_count += 1

    def push_to_hub(self, **kwargs: Any) -> None:
        self.pushes.append(kwargs)


def recording_config(tmp_path: Path, **overrides: Any) -> DatasetConfig:
    values = {
        "repo_id": "afterquery/test-yam-recording",
        "root": tmp_path / "dataset",
        "task": "Pick up the object",
        "top_camera_serial": "top-1",
        "left_camera_serial": "left-1",
        "right_camera_serial": "right-1",
        "min_episode_frames": 2,
    }
    values.update(overrides)
    return DatasetConfig(**values)


def ultrawide_recording_config(tmp_path: Path, **overrides: Any) -> DatasetConfig:
    values = {
        "side_camera_kind": "ultrawide",
        "left_camera_serial": None,
        "right_camera_serial": None,
        "left_camera_path": Path("/dev/v4l/by-path/left-wide"),
        "right_camera_path": Path("/dev/v4l/by-path/right-wide"),
    }
    values.update(overrides)
    return recording_config(tmp_path, **values)


def robot_values(offset: float = 0.0) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    return (
        {
            "left": np.arange(6, dtype=np.float64) + offset,
            "right": np.arange(6, dtype=np.float64) + 10.0 + offset,
        },
        {"left": 0.25 + offset / 100.0, "right": 0.75 + offset / 100.0},
    )


def recorder_fixture(
    tmp_path: Path,
    *,
    config: DatasetConfig | None = None,
    dataset: FakeDataset | None = None,
    camera_mutator: Callable[[dict[str, FakeCamera]], None] | None = None,
    control_hz: float = 30.0,
):
    dataset = dataset or FakeDataset()
    cameras = {name: FakeCamera(index) for index, name in enumerate(CAMERA_NAMES)}
    if camera_mutator is not None:
        camera_mutator(cameras)
    recorder = create_dataset_recorder(
        config or recording_config(tmp_path),
        control_hz,
        dataset_factory=lambda _config, _features: dataset,
        camera_factory=lambda _config: cameras,
        auth_validator=lambda: None,
    )
    assert recorder is not None
    return recorder, dataset, cameras


def test_dataset_sampling_decimates_100_hz_control_to_30_hz(tmp_path) -> None:
    recorder, _dataset, _cameras = recorder_fixture(tmp_path, control_hz=100.0)
    recorder.start_episode()
    decisions = [recorder.should_sample(index / 100.0) for index in range(100)]
    recorder.finish_episode(commit=False, reason="test")
    recorder.close()

    assert sum(decisions) == 30
    assert decisions[0]


def test_dataset_sampling_uses_elapsed_time_without_catchup_bursts(tmp_path) -> None:
    recorder, _dataset, _cameras = recorder_fixture(tmp_path, control_hz=120.0)
    recorder.start_episode()

    decisions = [
        recorder.should_sample(timestamp) for timestamp in (0.0, 0.010, 0.034, 0.200, 0.201, 0.234)
    ]

    recorder.finish_episode(commit=False, reason="test")
    recorder.close()
    assert decisions == [True, False, True, True, False, True]


def test_dataset_sampling_rejects_nonfinite_clock(tmp_path) -> None:
    recorder, _dataset, _cameras = recorder_fixture(tmp_path, control_hz=100.0)
    recorder.start_episode()

    with pytest.raises(DatasetRecordingError, match="sample time must be finite"):
        recorder.should_sample(float("nan"))

    recorder.finish_episode(commit=False, reason="test")
    recorder.close()


def add_frame(recorder, measured_offset: float = 0.0, command_offset: float = 0.5) -> None:
    measured_joints, measured_grippers = robot_values(measured_offset)
    commanded_joints, commanded_grippers = robot_values(command_offset)
    recorder.record_frame(
        measured_joints=measured_joints,
        measured_grippers=measured_grippers,
        commanded_joints=commanded_joints,
        commanded_grippers=commanded_grippers,
    )


def test_dataset_features_match_molmoact2_bimanual_yam_contract() -> None:
    features = dataset_features()

    assert tuple(features) == (
        "observation.state",
        "action",
        "observation.images.top",
        "observation.images.left",
        "observation.images.right",
    )
    assert len(YAM_SCALAR_NAMES) == 14
    assert features["observation.state"] == {
        "dtype": "float32",
        "shape": (14,),
        "names": list(YAM_SCALAR_NAMES),
    }
    assert features["action"] == features["observation.state"]
    for name in CAMERA_NAMES:
        feature = features[f"observation.images.{name}"]
        assert feature["dtype"] == "video"
        assert feature["shape"] == (270, 480, 3)


def test_ultrawide_dataset_features_keep_top_shape_and_expand_side_views() -> None:
    features = dataset_features("ultrawide")

    assert features["observation.images.top"]["shape"] == CAMERA_SHAPE
    assert features["observation.images.left"]["shape"] == ULTRAWIDE_CAMERA_SHAPE
    assert features["observation.images.right"]["shape"] == ULTRAWIDE_CAMERA_SHAPE
    assert dataset_camera_shapes("ultrawide") == {
        "top": CAMERA_SHAPE,
        "left": ULTRAWIDE_CAMERA_SHAPE,
        "right": ULTRAWIDE_CAMERA_SHAPE,
    }


def test_yam_vector_orders_left_gripper_then_right_gripper() -> None:
    joints, grippers = robot_values()

    vector = yam_vector(joints, grippers, label="test")

    assert vector.dtype == np.float32
    np.testing.assert_allclose(
        vector,
        [0, 1, 2, 3, 4, 5, 0.25, 10, 11, 12, 13, 14, 15, 0.75],
    )


def test_frame_keeps_measured_state_separate_from_applied_command() -> None:
    measured_joints, measured_grippers = robot_values(0.0)
    commanded_joints, commanded_grippers = robot_values(0.5)
    images = {name: FakeCamera(index).image for index, name in enumerate(CAMERA_NAMES)}

    frame = build_frame(
        images=images,
        measured_joints=measured_joints,
        measured_grippers=measured_grippers,
        commanded_joints=commanded_joints,
        commanded_grippers=commanded_grippers,
        task="Do the task",
    )

    assert frame["task"] == "Do the task"
    assert not np.array_equal(frame["observation.state"], frame["action"])
    np.testing.assert_allclose(frame["observation.state"][:6], np.arange(6))
    np.testing.assert_allclose(frame["action"][:6], np.arange(6) + 0.5)
    assert not np.shares_memory(frame["observation.images.top"], images["top"])


def test_top_camera_native_frame_is_center_cropped_and_downsampled() -> None:
    cv2 = pytest.importorskip("cv2")
    image = np.broadcast_to(
        np.arange(TOP_CAMERA_CAPTURE_HEIGHT, dtype=np.uint16)[:, None, None],
        (TOP_CAMERA_CAPTURE_HEIGHT, TOP_CAMERA_CAPTURE_WIDTH, 3),
    )
    image = (image % 256).astype(np.uint8)

    normalized = normalize_camera_image("top", image)

    crop_start = (TOP_CAMERA_CAPTURE_HEIGHT - TOP_CAMERA_CROP_HEIGHT) // 2
    cropped = image[crop_start : crop_start + TOP_CAMERA_CROP_HEIGHT]
    expected = cv2.resize(cropped, (480, 270), interpolation=cv2.INTER_AREA)
    assert normalized.shape == CAMERA_SHAPE
    np.testing.assert_array_equal(normalized, expected)
    assert normalized.flags.c_contiguous
    assert not np.shares_memory(normalized, image)


def test_ultrawide_camera_frame_is_downsampled_without_aspect_ratio_conversion() -> None:
    import cv2

    capture_shape = (ULTRAWIDE_CAPTURE_HEIGHT, ULTRAWIDE_CAPTURE_WIDTH, 3)
    image = np.arange(np.prod(capture_shape), dtype=np.uint8).reshape(capture_shape)

    normalized = normalize_camera_image("left", image, "ultrawide")
    expected = cv2.resize(
        image,
        (ULTRAWIDE_CAMERA_SHAPE[1], ULTRAWIDE_CAMERA_SHAPE[0]),
        interpolation=cv2.INTER_AREA,
    )

    assert normalized.shape == ULTRAWIDE_CAMERA_SHAPE
    np.testing.assert_array_equal(normalized, expected)
    assert normalized.flags.c_contiguous
    assert not np.shares_memory(normalized, image)


@pytest.mark.parametrize(
    ("images", "message"),
    [
        ({"top": np.zeros((1, 1, 3), dtype=np.uint8)}, "top, left, and right"),
        (
            {name: np.zeros((1, 1, 3), dtype=np.uint8) for name in CAMERA_NAMES},
            "does not match",
        ),
        (
            {name: np.zeros(CAMERA_SHAPE, dtype=np.float32) for name in CAMERA_NAMES},
            "RGB uint8",
        ),
    ],
)
def test_frame_rejects_missing_wrong_shape_and_wrong_dtype(images, message) -> None:
    joints, grippers = robot_values()
    with pytest.raises(DatasetFrameError, match=message):
        build_frame(
            images=images,
            measured_joints=joints,
            measured_grippers=grippers,
            commanded_joints=joints,
            commanded_grippers=grippers,
            task="task",
        )


def test_disabled_recording_does_not_call_factories() -> None:
    called = False

    def fail_factory(*_args):
        nonlocal called
        called = True
        raise AssertionError("factory should not run")

    assert (
        create_dataset_recorder(
            DatasetConfig(),
            30.0,
            dataset_factory=fail_factory,
            camera_factory=fail_factory,
        )
        is None
    )
    assert not called


def test_upload_authentication_is_checked_before_camera_or_dataset_creation(tmp_path) -> None:
    called: list[str] = []

    def reject_auth() -> None:
        called.append("auth")
        raise DatasetRecordingError("run `hf auth login`")

    def fail_factory(*_args):
        called.append("factory")
        raise AssertionError("factories must not run before authentication succeeds")

    with pytest.raises(DatasetRecordingError, match="hf auth login"):
        create_dataset_recorder(
            recording_config(tmp_path, push_to_hub=True),
            100.0,
            dataset_factory=fail_factory,
            camera_factory=fail_factory,
            auth_validator=reject_auth,
        )

    assert called == ["auth"]


def test_recording_reads_every_camera_before_creating_the_dataset(tmp_path) -> None:
    cameras = {name: FakeCamera(index) for index, name in enumerate(CAMERA_NAMES)}
    cameras["right"].image = np.zeros((1, 1, 3), dtype=np.uint8)
    dataset_created = False

    def make_dataset(*_args):
        nonlocal dataset_created
        dataset_created = True
        return FakeDataset()

    with pytest.raises(DatasetRecordingError, match="dataset camera preflight failed"):
        create_dataset_recorder(
            recording_config(tmp_path),
            100.0,
            dataset_factory=make_dataset,
            camera_factory=lambda _config: cameras,
        )

    assert not dataset_created
    assert all(camera.disconnected == 1 for camera in cameras.values())


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"repo_id": "missing-owner"}, "OWNER/NAME"),
        ({"task": ""}, "task must be nonempty"),
        ({"right_camera_serial": "left-1"}, "must be distinct"),
        ({"queue_size": 0}, "queue size must be positive"),
        ({"encoder_queue_size": 0}, "encoder queue size must be positive"),
        ({"encoder_gop_size": 0}, "encoder GOP size must be positive"),
        ({"min_free_disk_gb": 0}, "minimum free disk space must be positive"),
        ({"shutdown_timeout_s": 0}, "shutdown timeout must be positive"),
        ({"resume": True}, "resume requires an explicit local root"),
    ],
)
def test_recording_config_rejects_invalid_contract(tmp_path, overrides, message) -> None:
    if overrides.get("resume"):
        overrides["root"] = None
    with pytest.raises(DatasetRecordingError, match=message):
        recording_config(tmp_path, **overrides).validate(30.0)


@pytest.mark.parametrize(
    ("overrides", "message"),
    [
        ({"top_camera_serial": None}, "camera serial for top"),
        ({"left_camera_path": None}, "camera paths for left"),
        ({"right_camera_path": None}, "camera paths for right"),
        (
            {
                "left_camera_path": Path("/dev/video-wide"),
                "right_camera_path": Path("/dev/video-wide"),
            },
            "paths must be distinct",
        ),
    ],
)
def test_ultrawide_recording_config_requires_top_serial_and_distinct_side_paths(
    tmp_path: Path, overrides: dict[str, Any], message: str
) -> None:
    with pytest.raises(DatasetRecordingError, match=message):
        ultrawide_recording_config(tmp_path, **overrides).validate(30.0)


def test_camera_specs_preserve_realsense_defaults_and_add_ultrawide_option(tmp_path: Path) -> None:
    realsense_specs = _camera_specs(recording_config(tmp_path))
    assert [
        (spec.name, spec.backend, spec.width, spec.height, spec.requested_fps)
        for spec in realsense_specs
    ] == [
        ("top", "realsense", 640, 480, 30),
        ("left", "realsense", 480, 270, 30),
        ("right", "realsense", 480, 270, 30),
    ]

    ultrawide_specs = _camera_specs(ultrawide_recording_config(tmp_path))
    assert ultrawide_specs[0] == realsense_specs[0]
    for spec, name, path in zip(
        ultrawide_specs[1:],
        ("left", "right"),
        (Path("/dev/v4l/by-path/left-wide"), Path("/dev/v4l/by-path/right-wide")),
        strict=True,
    ):
        assert spec.name == name
        assert spec.backend == "opencv"
        assert spec.source == path
        assert (spec.width, spec.height) == (640, 480)
        assert spec.requested_fps == ULTRAWIDE_REQUESTED_FPS
        assert spec.fourcc == "MJPG"


def test_new_recording_rejects_nonempty_root(tmp_path: Path) -> None:
    root = tmp_path / "dataset"
    root.mkdir()
    (root / "existing.txt").write_text("occupied")

    with pytest.raises(DatasetRecordingError, match="root already exists"):
        recording_config(tmp_path, root=root).validate(30.0)


def test_streaming_h264_uses_realtime_encoder_defaults(tmp_path: Path) -> None:
    config = recording_config(tmp_path, streaming_encoding=True)

    assert config.encoder_gop_size == 30
    assert _effective_encoder_preset(config) == "ultrafast"
    assert _effective_encoder_preset(recording_config(tmp_path, encoder_preset="fast")) == "fast"
    assert (
        _effective_encoder_preset(
            recording_config(tmp_path, streaming_encoding=True, video_codec="libsvtav1")
        )
        is None
    )


def test_recording_checks_disk_before_starting_writer(tmp_path: Path) -> None:
    dataset = FakeDataset()
    cameras = {name: FakeCamera() for name in CAMERA_NAMES}
    config = recording_config(tmp_path, min_free_disk_gb=10**9)

    with pytest.raises(DatasetRecordingError, match="GiB free"):
        create_dataset_recorder(
            config,
            30.0,
            dataset_factory=lambda _config, _features: dataset,
            camera_factory=lambda _config: cameras,
        )

    assert dataset.finalize_count == 1
    assert all(camera.disconnected == 1 for camera in cameras.values())


def test_async_recorder_saves_only_intentional_long_enough_episode(tmp_path) -> None:
    recorder, dataset, cameras = recorder_fixture(tmp_path)

    assert recorder.queue_depths() == {"recorder": 0}

    recorder.start_episode()
    add_frame(recorder, 0.0)
    add_frame(recorder, 0.1)
    recorder.finish_episode(commit=True, reason="enable toggled off")

    recorder.start_episode()
    add_frame(recorder, 0.2)
    recorder.finish_episode(commit=True, reason="enable toggled off")

    recorder.start_episode()
    add_frame(recorder, 0.3)
    add_frame(recorder, 0.4)
    recorder.finish_episode(commit=False, reason="cancel button")
    recorder.close()

    assert len(dataset.episodes) == 1
    assert len(dataset.episodes[0]) == 2
    assert dataset.buffer == []
    assert dataset.clear_count == 2
    assert dataset.finalize_count == 1
    assert all(camera.disconnected == 1 for camera in cameras.values())


def test_ultrawide_recorder_writes_mixed_camera_shapes(tmp_path: Path) -> None:
    def use_ultrawide_side_frames(cameras: dict[str, FakeCamera]) -> None:
        cameras["left"].image = np.full(ULTRAWIDE_CAMERA_SHAPE, 1, dtype=np.uint8)
        cameras["right"].image = np.full(ULTRAWIDE_CAMERA_SHAPE, 2, dtype=np.uint8)

    recorder, dataset, _cameras = recorder_fixture(
        tmp_path,
        config=ultrawide_recording_config(tmp_path),
        camera_mutator=use_ultrawide_side_frames,
    )
    recorder.start_episode()
    add_frame(recorder)
    add_frame(recorder, measured_offset=0.1)
    recorder.finish_episode(commit=True, reason="enable toggled off")
    recorder.close()

    frame = dataset.episodes[0][0]
    assert frame["observation.images.top"].shape == CAMERA_SHAPE
    assert frame["observation.images.left"].shape == ULTRAWIDE_CAMERA_SHAPE
    assert frame["observation.images.right"].shape == ULTRAWIDE_CAMERA_SHAPE


def test_ultrawide_recorder_defers_side_downsampling_to_writer(tmp_path: Path) -> None:
    capture_shape = (ULTRAWIDE_CAPTURE_HEIGHT, ULTRAWIDE_CAPTURE_WIDTH, 3)

    def use_native_ultrawide_frames(cameras: dict[str, FakeCamera]) -> None:
        cameras["left"].image = np.full(capture_shape, 1, dtype=np.uint8)
        cameras["right"].image = np.full(capture_shape, 2, dtype=np.uint8)

    recorder, dataset, _cameras = recorder_fixture(
        tmp_path,
        config=ultrawide_recording_config(tmp_path),
        camera_mutator=use_native_ultrawide_frames,
    )
    measured_joints, measured_grippers = robot_values()
    commanded_joints, commanded_grippers = robot_values(0.5)
    recorder.start_episode()
    prepared = recorder.prepare_frame(
        measured_joints=measured_joints,
        measured_grippers=measured_grippers,
        commanded_joints=commanded_joints,
        commanded_grippers=commanded_grippers,
    )

    assert prepared["observation.images.left"].shape == capture_shape
    assert prepared["observation.images.right"].shape == capture_shape

    recorder.commit_frame(prepared)
    add_frame(recorder, measured_offset=0.1)
    recorder.finish_episode(commit=True, reason="enable toggled off")
    recorder.close()

    frame = dataset.episodes[0][0]
    assert frame["observation.images.left"].shape == ULTRAWIDE_CAMERA_SHAPE
    assert frame["observation.images.right"].shape == ULTRAWIDE_CAMERA_SHAPE


def test_streaming_encoder_drop_discards_episode_instead_of_saving(tmp_path) -> None:
    class DroppingDataset(FakeDataset):
        def __init__(self) -> None:
            super().__init__()
            encoder = type("Encoder", (), {"_dropped_frames": {}})()
            self.writer = type("Writer", (), {"_streaming_encoder": encoder})()

    dataset = DroppingDataset()
    recorder, dataset, _cameras = recorder_fixture(tmp_path, dataset=dataset)
    recorder.start_episode()
    add_frame(recorder)
    add_frame(recorder, measured_offset=0.1)
    deadline = time.monotonic() + 1.0
    while len(dataset.buffer) < 2 and time.monotonic() < deadline:
        time.sleep(0.005)
    assert len(dataset.buffer) == 2
    dataset.writer._streaming_encoder._dropped_frames = {"observation.images.left": 1}
    recorder.finish_episode(commit=True, reason="enable toggled off")
    recorder.close()

    assert dataset.episodes == []
    assert dataset.clear_count == 1
    assert dataset.writer._streaming_encoder._dropped_frames == {}


def test_streaming_encoder_drop_invalidates_active_frame(tmp_path) -> None:
    dataset = FakeDataset()
    encoder = type(
        "Encoder",
        (),
        {"_dropped_frames": {"observation.images.right": 2}},
    )()
    dataset.writer = type("Writer", (), {"_streaming_encoder": encoder})()
    recorder, _dataset, _cameras = recorder_fixture(tmp_path, dataset=dataset)
    recorder.start_episode()

    with pytest.raises(DatasetFrameError, match="observation.images.right=2"):
        add_frame(recorder)
    recorder.finish_episode(commit=False, reason="encoder drop")
    recorder.close()


def test_finalized_dataset_rejects_mismatched_encoded_video_count(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "dataset"
    root.mkdir()
    video_keys = [f"observation.images.{name}" for name in CAMERA_NAMES]
    relative_paths = {
        key: Path("videos") / key / "chunk-000" / "file-000.mp4" for key in video_keys
    }
    for path in relative_paths.values():
        target = root / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.touch()

    class MediaMeta:
        features = dataset_features()
        fps = 30
        total_episodes = 1
        total_frames = 10
        episodes = [
            {
                "length": 10,
                "dataset_from_index": 0,
                "dataset_to_index": 10,
            }
        ]

        @staticmethod
        def get_video_file_path(_episode_index: int, video_key: str) -> Path:
            return relative_paths[video_key]

    dataset = type("FinalizedDataset", (), {"root": root, "meta": MediaMeta()})()
    monkeypatch.setattr(
        "umi_teleop.dataset._video_frame_count",
        lambda path: 9 if "left" in str(path) else 10,
    )

    with pytest.raises(
        DatasetRecordingError,
        match=r"observation.images.left has 9 encoded frames; expected 10",
    ):
        validate_finalized_dataset(dataset)


def test_finalized_dataset_accepts_ultrawide_schema() -> None:
    meta = type(
        "Meta",
        (),
        {
            "features": dataset_features("ultrawide"),
            "fps": 30,
            "total_episodes": 1,
            "total_frames": 1,
        },
    )()
    dataset = type("FinalizedDataset", (), {"meta": meta})()

    validate_finalized_dataset(dataset)


def test_finalized_dataset_accepts_shared_video_files_across_episodes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    root = tmp_path / "dataset"
    root.mkdir()
    video_keys = [f"observation.images.{name}" for name in CAMERA_NAMES]
    relative_paths = {
        key: Path("videos") / key / "chunk-000" / "file-000.mp4" for key in video_keys
    }
    for path in relative_paths.values():
        target = root / path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.touch()

    class SharedMediaMeta:
        features = dataset_features()
        fps = 30
        total_episodes = 2
        total_frames = 15
        episodes = [
            {"length": 10, "dataset_from_index": 0, "dataset_to_index": 10},
            {"length": 5, "dataset_from_index": 10, "dataset_to_index": 15},
        ]

        @staticmethod
        def get_video_file_path(_episode_index: int, video_key: str) -> Path:
            return relative_paths[video_key]

    dataset = type("FinalizedDataset", (), {"root": root, "meta": SharedMediaMeta()})()
    inspected: list[Path] = []

    def frame_count(path: Path) -> int:
        inspected.append(path)
        return 15

    monkeypatch.setattr("umi_teleop.dataset._video_frame_count", frame_count)

    validate_finalized_dataset(dataset)

    assert sorted(inspected) == sorted(root / path for path in relative_paths.values())


def test_enable_starts_episode_when_no_startup_episode_is_active(tmp_path) -> None:
    recorder, dataset, _cameras = recorder_fixture(tmp_path)
    measured_joints, measured_grippers = robot_values(0.25)
    commanded_joints, commanded_grippers = robot_values(0.5)

    prepared = prepare_dataset_step(
        recorder,
        was_armed=False,
        is_armed=True,
        invalid_frame_reason=None,
        measured_joints=measured_joints,
        measured_grippers=measured_grippers,
        commanded_joints=commanded_joints,
        commanded_grippers=commanded_grippers,
    )
    commit_dataset_step(
        recorder,
        prepared,
        was_armed=False,
        is_armed=True,
        disarm_reason="enabled",
    )
    prepared = prepare_dataset_step(
        recorder,
        was_armed=True,
        is_armed=True,
        invalid_frame_reason=None,
        measured_joints=measured_joints,
        measured_grippers=measured_grippers,
        commanded_joints=commanded_joints,
        commanded_grippers=commanded_grippers,
    )
    commit_dataset_step(
        recorder,
        prepared,
        was_armed=True,
        is_armed=True,
        disarm_reason="enabled",
    )
    commit_dataset_step(
        recorder,
        None,
        was_armed=True,
        is_armed=False,
        disarm_reason="enable toggled off",
    )
    recorder.close()

    assert len(dataset.episodes) == 1
    assert len(dataset.episodes[0]) == 2


def test_enable_continues_episode_that_started_during_ready_motion(tmp_path) -> None:
    recorder, dataset, _cameras = recorder_fixture(tmp_path)
    recorder.start_episode()
    add_frame(recorder, measured_offset=0.0, command_offset=0.25)
    measured_joints, measured_grippers = robot_values(0.25)
    commanded_joints, commanded_grippers = robot_values(0.5)

    prepared = prepare_dataset_step(
        recorder,
        was_armed=False,
        is_armed=True,
        invalid_frame_reason=None,
        measured_joints=measured_joints,
        measured_grippers=measured_grippers,
        commanded_joints=commanded_joints,
        commanded_grippers=commanded_grippers,
    )
    commit_dataset_step(
        recorder,
        prepared,
        was_armed=False,
        is_armed=True,
        disarm_reason="enabled",
    )
    commit_dataset_step(
        recorder,
        None,
        was_armed=True,
        is_armed=False,
        disarm_reason="enable toggled off",
    )
    recorder.close()

    assert len(dataset.episodes) == 1
    assert len(dataset.episodes[0]) == 2


def test_push_happens_after_finalize_and_is_private_by_default(tmp_path) -> None:
    events: list[str] = []

    class OrderedDataset(FakeDataset):
        def finalize(self) -> None:
            events.append("finalize")
            super().finalize()

        def push_to_hub(self, **kwargs: Any) -> None:
            events.append("push")
            super().push_to_hub(**kwargs)

    config = recording_config(tmp_path, push_to_hub=True)
    recorder, dataset, _cameras = recorder_fixture(
        tmp_path, config=config, dataset=OrderedDataset()
    )

    recorder.start_episode()
    add_frame(recorder)
    add_frame(recorder, measured_offset=0.1)
    recorder.finish_episode(commit=True, reason="enable toggled off")
    recorder.close()

    assert events == ["finalize", "push"]
    assert dataset.pushes == [{"private": True}]


def test_upload_can_be_deferred_after_controller_cleanup_error(tmp_path) -> None:
    config = recording_config(tmp_path, push_to_hub=True)
    recorder, dataset, _cameras = recorder_fixture(tmp_path, config=config)
    recorder.start_episode()
    add_frame(recorder)
    add_frame(recorder, measured_offset=0.1)
    recorder.finish_episode(commit=True, reason="enable toggled off")

    recorder.close(upload=False)

    assert len(dataset.episodes) == 1
    assert dataset.finalize_count == 1
    assert dataset.pushes == []


def test_camera_failure_discards_active_episode_without_killing_writer(tmp_path) -> None:
    recorder, dataset, cameras = recorder_fixture(tmp_path)
    recorder.start_episode()
    cameras["top"].image = np.zeros((1, 1, 3), dtype=np.uint8)

    with pytest.raises(DatasetFrameError, match="top camera frame shape"):
        add_frame(recorder)
    recorder.finish_episode(commit=False, reason="camera frame invalid")
    recorder.close()

    assert dataset.episodes == []
    assert dataset.clear_count == 1
    assert dataset.finalize_count == 1


def test_disconnected_camera_is_reported_as_episode_frame_error(tmp_path) -> None:
    class DisconnectedCamera(FakeCamera):
        def read_latest(self, max_age_ms: int = 500) -> np.ndarray:
            raise ConnectionError("camera disconnected")

    recorder, _dataset, _cameras = recorder_fixture(tmp_path)
    recorder.cameras["top"] = DisconnectedCamera()
    recorder.start_episode()

    with pytest.raises(DatasetFrameError, match="camera disconnected"):
        add_frame(recorder)
    recorder.finish_episode(commit=False, reason="camera disconnected")
    recorder.close()


def test_prepared_frame_is_not_written_until_bimanual_dispatch_commits(tmp_path) -> None:
    recorder, dataset, _cameras = recorder_fixture(tmp_path)
    measured_joints, measured_grippers = robot_values(0.0)
    commanded_joints, commanded_grippers = robot_values(0.5)
    recorder.start_episode()

    recorder.prepare_frame(
        measured_joints=measured_joints,
        measured_grippers=measured_grippers,
        commanded_joints=commanded_joints,
        commanded_grippers=commanded_grippers,
    )
    recorder.finish_episode(commit=False, reason="right arm dispatch failed")
    recorder.close()

    assert dataset.episodes == []
    assert dataset.buffer == []


def test_background_writer_failure_is_reported_and_finalized(tmp_path) -> None:
    recorder, dataset, cameras = recorder_fixture(tmp_path, dataset=FakeDataset(fail_add=True))
    recorder.start_episode()
    add_frame(recorder)

    with pytest.raises(DatasetRecordingError, match="disk full"):
        recorder.close()

    assert dataset.episodes == []
    assert dataset.finalize_count == 1
    assert all(camera.disconnected == 1 for camera in cameras.values())


def test_queue_overflow_fails_and_close_does_not_deadlock(tmp_path) -> None:
    entered = Event()
    release = Event()

    class BlockingDataset(FakeDataset):
        def add_frame(self, frame: dict[str, Any]) -> None:
            entered.set()
            assert release.wait(timeout=2.0)
            super().add_frame(frame)

    config = recording_config(tmp_path, queue_size=1, min_episode_frames=1)
    recorder, dataset, _cameras = recorder_fixture(
        tmp_path,
        config=config,
        dataset=BlockingDataset(),
    )
    recorder.start_episode()
    add_frame(recorder)
    assert entered.wait(timeout=1.0)
    add_frame(recorder, measured_offset=0.1)

    with pytest.raises(DatasetRecordingError, match="queue is full"):
        add_frame(recorder, measured_offset=0.2)
    release.set()
    with pytest.raises(DatasetRecordingError, match="queue is full"):
        recorder.close()

    assert dataset.finalize_count == 1


def test_writer_backlog_invalidates_episode_before_fatal_queue_overflow(tmp_path) -> None:
    entered = Event()
    release = Event()

    class BlockingDataset(FakeDataset):
        def add_frame(self, frame: dict[str, Any]) -> None:
            entered.set()
            assert release.wait(timeout=2.0)
            super().add_frame(frame)

    config = recording_config(tmp_path, queue_size=10, min_episode_frames=1)
    recorder, dataset, _cameras = recorder_fixture(
        tmp_path,
        config=config,
        dataset=BlockingDataset(),
    )
    recorder.start_episode()
    add_frame(recorder)
    assert entered.wait(timeout=1.0)
    for offset in np.linspace(0.1, 0.8, 8):
        add_frame(recorder, measured_offset=float(offset))

    with pytest.raises(DatasetFrameError, match="writer backlog reached 8/10"):
        add_frame(recorder, measured_offset=0.9)

    recorder.finish_episode(commit=False, reason="writer backlog")
    release.set()
    recorder.close()

    assert dataset.episodes == []
    assert dataset.clear_count == 1
    assert dataset.finalize_count == 1


def test_hung_writer_has_bounded_shutdown(tmp_path) -> None:
    entered = Event()
    release = Event()

    class HungDataset(FakeDataset):
        def add_frame(self, frame: dict[str, Any]) -> None:
            entered.set()
            release.wait()
            super().add_frame(frame)

    config = recording_config(tmp_path, shutdown_timeout_s=0.05)
    recorder, dataset, _cameras = recorder_fixture(
        tmp_path,
        config=config,
        dataset=HungDataset(),
    )
    recorder.start_episode()
    add_frame(recorder)
    assert entered.wait(timeout=1.0)

    started = time.monotonic()
    with pytest.raises(DatasetRecordingError, match="did not stop within"):
        recorder.close()
    elapsed = time.monotonic() - started
    release.set()
    recorder._thread.join(timeout=1.0)

    assert elapsed < 1.0
    assert not recorder._thread.is_alive()
    assert dataset.finalize_count == 0


def test_close_discards_incomplete_episode_and_is_idempotent(tmp_path) -> None:
    recorder, dataset, _cameras = recorder_fixture(tmp_path)
    recorder.start_episode()
    add_frame(recorder)
    add_frame(recorder)

    recorder.close()
    recorder.close()

    assert dataset.episodes == []
    assert dataset.clear_count == 1
    assert dataset.finalize_count == 1

from __future__ import annotations

import json
import socket
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from umi_teleop.controller import GripperMap, TeleopLimits, TeleopStateMachine, UdpTeleopReceiver
from umi_teleop.dataset import (
    CAMERA_NAMES,
    CAMERA_SHAPE,
    ULTRAWIDE_CAMERA_SHAPE,
    DatasetConfig,
    DatasetRecordingError,
    create_dataset_recorder,
    dataset_features,
    update_dataset_episode,
)
from umi_teleop.protocol import ButtonState, encode_teleop_packet
from umi_teleop.types import Event


class IntegrationCamera:
    def __init__(self, value: int, shape: tuple[int, int, int] = CAMERA_SHAPE) -> None:
        self.value = value
        self.shape = shape
        self.disconnected = False

    def read_latest(self, max_age_ms: int = 500) -> np.ndarray:
        assert max_age_ms == 250
        self.value += 1
        return np.full(self.shape, self.value, dtype=np.uint8)

    def disconnect(self) -> None:
        self.disconnected = True


class DiskDataset:
    def __init__(self, root: Path, features: dict[str, Any]) -> None:
        self.root = root
        self.meta = type("Meta", (), {"features": features})()
        self.buffer: list[dict[str, Any]] = []
        self.episodes: list[list[dict[str, Any]]] = []
        self.finalized = False

    def add_frame(self, frame: dict[str, Any]) -> None:
        self.buffer.append(frame)

    def save_episode(self) -> None:
        self.episodes.append(self.buffer)
        self.buffer = []

    def clear_episode_buffer(self, delete_images: bool = True) -> None:
        assert delete_images
        self.buffer = []

    def finalize(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        summary = {
            "episodes": len(self.episodes),
            "frames": [len(episode) for episode in self.episodes],
            "tasks": [episode[0]["task"] for episode in self.episodes],
        }
        (self.root / "manifest.json").write_text(json.dumps(summary))
        self.finalized = True

    def push_to_hub(self, **_kwargs: Any) -> None:
        raise AssertionError("integration test must remain local")


def source_packet(state: ButtonState, seq: int) -> dict[str, Any]:
    pose = {
        "left": {"serial": "L", "pose": [0, 0, 0, 0, 0, 0, 1], "age_s": 0.01},
        "right": {"serial": "R", "pose": [0, 0, 0, 0, 0, 0, 1], "age_s": 0.01},
    }
    widths = {
        "left": {"width_mm": 80.0, "age_s": 0.01},
        "right": {"width_mm": 80.0, "age_s": 0.01},
    }
    return state.packet(seq, pose, widths)


def udp_step(
    sock: socket.socket,
    receiver: UdpTeleopReceiver,
    machine: TeleopStateMachine,
    packet: dict[str, Any],
):
    sock.sendto(encode_teleop_packet(packet), ("127.0.0.1", receiver.port))
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        received, age = receiver.poll()
        if received is not None and received["seq"] == packet["seq"]:
            assert age is not None
            return machine.step(
                received,
                age,
                {"left": np.eye(4), "right": np.eye(4)},
            )
        time.sleep(0.005)
    raise AssertionError("packet did not cross the localhost UDP boundary")


def record_result(recorder, result, was_armed: bool, offset: float) -> None:
    measured_joints = {
        "left": np.full(6, offset),
        "right": np.full(6, offset + 1.0),
    }
    measured_grippers = {"left": 0.25, "right": 0.75}
    commanded_joints = {
        "left": np.full(6, offset + 0.1),
        "right": np.full(6, offset + 1.1),
    }
    commanded_grippers = {"left": 0.3, "right": 0.7}
    update_dataset_episode(
        recorder,
        was_armed=was_armed,
        is_armed=result.armed,
        disarm_reason=result.reason,
        invalid_frame_reason=None,
        measured_joints=measured_joints,
        measured_grippers=measured_grippers,
        commanded_joints=commanded_joints,
        commanded_grippers=commanded_grippers,
    )


@pytest.mark.integration
def test_udp_enable_stop_and_cancel_drive_dataset_episode_lifecycle(tmp_path: Path) -> None:
    receiver = UdpTeleopReceiver("127.0.0.1", 0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    machine = TeleopStateMachine(
        {"left": GripperMap(40, 120), "right": GripperMap(40, 120)},
        TeleopLimits(max_translation_m=0.4),
    )
    state = ButtonState(session_id="dataset-integration")
    config = DatasetConfig(
        repo_id="afterquery/udp-integration",
        root=tmp_path / "dataset",
        task="Move the object",
        top_camera_serial="top",
        left_camera_serial="left",
        right_camera_serial="right",
        min_episode_frames=2,
    )
    cameras = {name: IntegrationCamera(index) for index, name in enumerate(CAMERA_NAMES)}
    disk_dataset: DiskDataset | None = None

    def make_dataset(_config, features):
        nonlocal disk_dataset
        disk_dataset = DiskDataset(config.root, features)
        return disk_dataset

    recorder = create_dataset_recorder(
        config,
        30.0,
        dataset_factory=make_dataset,
        camera_factory=lambda _config: cameras,
    )
    assert recorder is not None

    try:
        assert not udp_step(sock, receiver, machine, source_packet(state, 0)).armed
        state.handle(Event.ZERO)
        assert udp_step(sock, receiver, machine, source_packet(state, 1)).zeroed

        state.handle(Event.START)
        first = udp_step(sock, receiver, machine, source_packet(state, 2))
        record_result(recorder, first, was_armed=False, offset=0.0)
        second = udp_step(sock, receiver, machine, source_packet(state, 3))
        record_result(recorder, second, was_armed=True, offset=0.1)

        state.handle(Event.STOP)
        stopped = udp_step(sock, receiver, machine, source_packet(state, 4))
        record_result(recorder, stopped, was_armed=True, offset=0.2)

        state.handle(Event.START)
        restarted = udp_step(sock, receiver, machine, source_packet(state, 5))
        record_result(recorder, restarted, was_armed=False, offset=0.3)
        state.handle(Event.CANCEL)
        cancelled = udp_step(sock, receiver, machine, source_packet(state, 6))
        record_result(recorder, cancelled, was_armed=True, offset=0.4)
    finally:
        recorder.close()
        receiver.close()
        sock.close()

    assert disk_dataset is not None and disk_dataset.finalized
    manifest = json.loads((config.root / "manifest.json").read_text())
    assert manifest == {"episodes": 1, "frames": [2], "tasks": ["Move the object"]}
    assert all(camera.disconnected for camera in cameras.values())


@pytest.mark.integration
def test_real_lerobot_v3_video_dataset_can_finalize_and_reopen(tmp_path: Path) -> None:
    lerobot_dataset = pytest.importorskip("lerobot.datasets.lerobot_dataset")
    config = DatasetConfig(
        repo_id="afterquery/umi-teleop-local-integration",
        root=tmp_path / "lerobot",
        task="Move the object",
        top_camera_serial="top",
        left_camera_serial="left",
        right_camera_serial="right",
        min_episode_frames=2,
        video_codec="h264",
    )
    cameras = {name: IntegrationCamera(index) for index, name in enumerate(CAMERA_NAMES)}
    recorder = create_dataset_recorder(
        config,
        30.0,
        camera_factory=lambda _config: cameras,
    )
    assert recorder is not None
    recorder.start_episode()
    for offset in (0.0, 0.1, 0.2):
        measured = {
            "left": np.full(6, offset),
            "right": np.full(6, offset + 1.0),
        }
        recorder.record_frame(
            measured_joints=measured,
            measured_grippers={"left": 0.25, "right": 0.75},
            commanded_joints={name: values + 0.1 for name, values in measured.items()},
            commanded_grippers={"left": 0.3, "right": 0.7},
        )
    recorder.finish_episode(commit=True, reason="enable toggled off")
    recorder.close()

    reopened = lerobot_dataset.LeRobotDataset(
        config.repo_id, root=config.root, video_backend="pyav"
    )
    assert reopened.meta.total_episodes == 1
    assert reopened.meta.total_frames == 3
    sample = reopened[0]
    np.testing.assert_allclose(sample["observation.state"][:6], np.zeros(6))
    assert all(f"observation.images.{name}" in sample for name in CAMERA_NAMES)

    resumed = create_dataset_recorder(
        replace(config, resume=True),
        30.0,
        camera_factory=lambda _config: {
            name: IntegrationCamera(index + 10) for index, name in enumerate(CAMERA_NAMES)
        },
    )
    assert resumed is not None
    resumed.start_episode()
    for offset in (0.3, 0.4):
        measured = {
            "left": np.full(6, offset),
            "right": np.full(6, offset + 1.0),
        }
        resumed.record_frame(
            measured_joints=measured,
            measured_grippers={"left": 0.25, "right": 0.75},
            commanded_joints={name: values + 0.1 for name, values in measured.items()},
            commanded_grippers={"left": 0.3, "right": 0.7},
        )
    resumed.finish_episode(commit=True, reason="enable toggled off")
    resumed.close()

    appended = lerobot_dataset.LeRobotDataset(
        config.repo_id, root=config.root, video_backend="pyav"
    )
    assert appended.meta.total_episodes == 2
    assert appended.meta.total_frames == 5
    np.testing.assert_allclose(appended[4]["observation.state"][:6], np.full(6, 0.4))

    wrong_fps_root = tmp_path / "wrong-fps"
    wrong_fps = lerobot_dataset.LeRobotDataset.create(
        "afterquery/wrong-fps",
        15,
        features=dataset_features(),
        root=wrong_fps_root,
        use_videos=True,
    )
    wrong_fps.finalize()
    with pytest.raises(DatasetRecordingError, match="frame rate"):
        create_dataset_recorder(
            replace(config, repo_id="afterquery/wrong-fps", root=wrong_fps_root, resume=True),
            30.0,
            camera_factory=lambda _config: {
                name: IntegrationCamera(index + 20) for index, name in enumerate(CAMERA_NAMES)
            },
        )


@pytest.mark.integration
def test_real_lerobot_v3_video_dataset_supports_ultrawide_side_shapes(tmp_path: Path) -> None:
    lerobot_dataset = pytest.importorskip("lerobot.datasets.lerobot_dataset")
    video_utils = pytest.importorskip("lerobot.datasets.video_utils")
    config = DatasetConfig(
        repo_id="afterquery/umi-teleop-ultrawide-integration",
        root=tmp_path / "lerobot-ultrawide",
        task="Move the object",
        top_camera_serial="top",
        side_camera_kind="ultrawide",
        left_camera_path=Path("/dev/v4l/by-path/left-wide"),
        right_camera_path=Path("/dev/v4l/by-path/right-wide"),
        min_episode_frames=2,
        video_codec="h264",
        streaming_encoding=True,
    )
    cameras = {
        name: IntegrationCamera(index, config.camera_shapes[name])
        for index, name in enumerate(CAMERA_NAMES)
    }
    recorder = create_dataset_recorder(
        config,
        30.0,
        camera_factory=lambda _config: cameras,
    )
    assert recorder is not None
    assert getattr(video_utils._CameraEncoderThread, "_umi_deferred_rgb_stats", False)
    recorder.start_episode()
    for offset in (0.0, 0.1, 0.2):
        measured = {
            "left": np.full(6, offset),
            "right": np.full(6, offset + 1.0),
        }
        recorder.record_frame(
            measured_joints=measured,
            measured_grippers={"left": 0.25, "right": 0.75},
            commanded_joints={name: values + 0.1 for name, values in measured.items()},
            commanded_grippers={"left": 0.3, "right": 0.7},
        )
    recorder.finish_episode(commit=True, reason="enable toggled off")
    recorder.close()

    reopened = lerobot_dataset.LeRobotDataset(
        config.repo_id,
        root=config.root,
        video_backend="pyav",
        return_uint8=True,
    )
    assert reopened.meta.total_frames == 3
    assert reopened.meta.features["observation.images.top"]["shape"] == CAMERA_SHAPE
    assert reopened.meta.features["observation.images.left"]["shape"] == ULTRAWIDE_CAMERA_SHAPE
    sample = reopened[0]
    for name, expected_shape in config.camera_shapes.items():
        actual_shape = tuple(sample[f"observation.images.{name}"].shape)
        channel_first_shape = (expected_shape[2], expected_shape[0], expected_shape[1])
        assert actual_shape in (expected_shape, channel_first_shape)

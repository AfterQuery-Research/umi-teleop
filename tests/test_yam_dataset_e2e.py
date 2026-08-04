from __future__ import annotations

import socket
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import pytest

import umi_teleop.yam as yam
from umi_teleop.dataset import (
    CAMERA_NAMES,
    CAMERA_SHAPE,
    DatasetConfig,
    DatasetFrameError,
    create_dataset_recorder,
)
from umi_teleop.protocol import ButtonState, encode_teleop_packet
from umi_teleop.types import Event


class E2ECamera:
    def __init__(self, value: int) -> None:
        self.image = np.full(CAMERA_SHAPE, value, dtype=np.uint8)
        self.disconnected = False

    def read_latest(self, max_age_ms: int = 500) -> np.ndarray:
        assert max_age_ms > 0
        return self.image

    def disconnect(self) -> None:
        self.disconnected = True


class E2EDataset:
    def __init__(self, features: dict[str, Any]) -> None:
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
        self.finalized = True

    def push_to_hub(self, **_kwargs: Any) -> None:
        raise AssertionError("E2E test must remain local")


class E2ERobot:
    def __init__(self) -> None:
        self.joints = np.asarray(yam.DEFAULT_SIM_INITIAL_JOINTS, dtype=np.float64)
        self.gripper = 0.5
        self.commands: list[np.ndarray] = []
        self.closed = False

    def get_observations(self) -> dict[str, Any]:
        return {"joint_pos": self.joints.copy(), "gripper_pos": np.array([self.gripper])}

    def get_robot_info(self) -> dict[str, Any]:
        return {"joint_limits": np.tile(np.array([[-4.0, 4.0]]), (6, 1))}

    def command_joint_pos(self, target: np.ndarray) -> None:
        command = np.asarray(target, dtype=np.float64).copy()
        self.commands.append(command)
        self.joints = command[:6]
        self.gripper = float(command[6])

    def close(self) -> None:
        self.closed = True


class E2EIk:
    def __init__(self, *_args: Any) -> None:
        pass

    def fk(self, _joints: np.ndarray) -> np.ndarray:
        return np.eye(4)

    def solve(self, _target: np.ndarray, initial: np.ndarray) -> tuple[bool, np.ndarray]:
        time.sleep(0.05)
        return True, np.asarray(initial, dtype=np.float64) + 0.005


def test_startup_camera_failure_prevents_arm_dispatch() -> None:
    robots = {"left": E2ERobot(), "right": E2ERobot()}
    joints = {name: robot.joints.copy() for name, robot in robots.items()}
    grips = {name: robot.gripper for name, robot in robots.items()}

    class FailingRecorder:
        def prepare_frame(self, **_kwargs: Any) -> dict[str, Any]:
            raise DatasetFrameError("camera unavailable")

        def commit_frame(self, _frame: dict[str, Any]) -> None:
            raise AssertionError("a failed startup frame must not be committed")

    with pytest.raises(DatasetFrameError, match="camera unavailable"):
        yam.command_startup_pair(robots, joints, grips, FailingRecorder())

    assert all(robot.commands == [] for robot in robots.values())


def free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def source_packet(state: ButtonState, seq: int) -> dict[str, Any]:
    poses = {
        "left": {"serial": "L", "pose": [0, 0, 0, 0, 0, 0, 1], "age_s": 0.01},
        "right": {"serial": "R", "pose": [0, 0, 0, 0, 0, 0, 1], "age_s": 0.01},
    }
    widths = {
        "left": {"width_mm": 80.0, "age_s": 0.01},
        "right": {"width_mm": 90.0, "age_s": 0.01},
    }
    return state.packet(seq, poses, widths)


def stream_source(port: int, done: threading.Event) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    state = ButtonState(session_id="yam-recording-e2e")
    seq = 0

    def send(count: int) -> None:
        nonlocal seq
        for _ in range(count):
            sock.sendto(
                encode_teleop_packet(source_packet(state, seq)),
                ("127.0.0.1", port),
            )
            seq += 1
            time.sleep(0.035)

    try:
        send(6)
        state.handle(Event.ZERO)
        send(4)
        state.handle(Event.START)
        send(8)
        state.handle(Event.STOP)
        send(4)
        state.handle(Event.QUIT)
        send(3)
    finally:
        done.set()
        sock.close()


@pytest.mark.integration
def test_production_yam_loop_records_final_dispatched_simulated_actions(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    port = free_udp_port()
    dataset_config = DatasetConfig(
        repo_id="afterquery/yam-production-e2e",
        root=tmp_path / "dataset",
        task="Move the object",
        top_camera_serial="top",
        left_camera_serial="left",
        right_camera_serial="right",
        min_episode_frames=2,
        min_free_disk_gb=0.001,
    )
    dataset: E2EDataset | None = None
    cameras = {name: E2ECamera(index) for index, name in enumerate(CAMERA_NAMES)}

    def make_dataset(_config, features):
        nonlocal dataset
        dataset = E2EDataset(features)
        return dataset

    recorder = create_dataset_recorder(
        dataset_config,
        30.0,
        dataset_factory=make_dataset,
        camera_factory=lambda _config: cameras,
    )
    assert recorder is not None
    robots: list[E2ERobot] = []

    def make_robot(*_args: Any, **_kwargs: Any) -> E2ERobot:
        robot = E2ERobot()
        robots.append(robot)
        return robot

    monkeypatch.setattr(yam, "create_dataset_recorder", lambda *_args, **_kwargs: recorder)
    monkeypatch.setattr(yam, "IkModel", E2EIk)
    monkeypatch.setattr(yam, "make_robot", make_robot)
    monkeypatch.setattr(yam.signal, "signal", lambda *_args: None)

    sender_done = threading.Event()
    sender = threading.Thread(target=stream_source, args=(port, sender_done), daemon=True)
    sender.start()
    yam.run(
        yam.Config(
            bind="127.0.0.1",
            port=port,
            move_to_ready=True,
            ready_joint_speed=100.0,
            simulate=True,
            stream_timeout=2.0,
            dataset=dataset_config,
        )
    )
    sender.join(timeout=2.0)

    assert sender_done.is_set()
    assert dataset is not None and dataset.finalized
    assert len(dataset.episodes) == 1
    assert len(dataset.episodes[0]) >= 6
    assert len(robots) == 2 and all(robot.closed for robot in robots)
    assert all(camera.disconnected for camera in cameras.values())

    for frame in dataset.episodes[0]:
        action = frame["action"]
        assert any(np.allclose(action[:7], command) for command in robots[0].commands)
        assert any(np.allclose(action[7:], command) for command in robots[1].commands)
    assert any(
        not np.array_equal(frame["observation.state"], frame["action"])
        for frame in dataset.episodes[0]
    )
    first_frame = dataset.episodes[0][0]
    np.testing.assert_allclose(
        first_frame["observation.state"][:6],
        yam.DEFAULT_SIM_INITIAL_JOINTS,
    )
    np.testing.assert_allclose(first_frame["action"][:6], yam.DEFAULT_READY_JOINTS)
    assert any(
        not np.allclose(frame["action"][:6], yam.DEFAULT_READY_JOINTS)
        for frame in dataset.episodes[0][4:]
    )

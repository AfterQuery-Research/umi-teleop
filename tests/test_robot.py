from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np
import pytest

from umi_teleop_fixed.calibration import (
    GripperCalibration,
    calibration_path,
    save_gripper_calibration,
)
from umi_teleop_fixed.can import CANInterfaceInfo
from umi_teleop_fixed.robot import (
    DEFAULT_START_POSITION,
    ArmState,
    DualYamBackend,
    I2RTArm,
    YamKinematics,
    validate_dual_position,
)


def _ready(name: str, serial: str) -> CANInterfaceInfo:
    return CANInterfaceInfo(
        name=name,
        adapter_serial=serial,
        driver="gs_usb",
        is_up=True,
        lower_up=True,
        operating_state="UP",
        can_state="ERROR-ACTIVE",
        bitrate=1_000_000,
        is_fd=False,
    )


def _save_calibrations(root):
    path = calibration_path("robot-01", root=root)
    for side, serial, limits in (
        ("left", "LEFT", (6.5, 1.2)),
        ("right", "RIGHT", (6.4, 1.1)),
    ):
        save_gripper_calibration(
            path,
            robot_id="robot-01",
            side=side,
            calibration=GripperCalibration(serial, "linear_4310", limits, "now"),
        )


@dataclass
class FakeArm:
    commands: list = field(default_factory=list)
    idle_count: int = 0
    closed: bool = False

    def read_state(self):
        return ArmState(np.zeros(6), 0.4, np.eye(4), 1.0)

    def solve_ik(self, _target, seed):
        return seed + 0.1

    def command(self, joints, gripper):
        self.commands.append((joints.copy(), gripper))

    def idle(self):
        self.idle_count += 1

    def close(self):
        self.closed = True


def test_open_resolves_stable_serials_and_cached_limits(tmp_path):
    _save_calibrations(tmp_path)
    created = []

    def factory(channel, limits):
        arm = FakeArm()
        created.append((channel, limits, arm))
        return arm

    backend = DualYamBackend.open(
        robot_id="robot-01",
        left_adapter_serial="LEFT",
        right_adapter_serial="RIGHT",
        calibration_root=tmp_path,
        arm_factory=factory,
        interfaces=[_ready("can0", "RIGHT"), _ready("can1", "LEFT")],
    )

    assert [(channel, limits) for channel, limits, _arm in created] == [
        ("can1", (6.5, 1.2)),
        ("can0", (6.4, 1.1)),
    ]
    states = backend.read_states()
    solutions = backend.solve_ik(
        {"left": np.eye(4), "right": np.eye(4)},
        {side: state.joint_positions for side, state in states.items()},
    )
    backend.command(solutions, {"left": 0.4, "right": 0.4})
    backend.close()
    assert all(arm.commands for _channel, _limits, arm in created)
    assert all(arm.idle_count == 1 and arm.closed for _channel, _limits, arm in created)


def test_open_rejects_calibration_for_different_adapter(tmp_path):
    _save_calibrations(tmp_path)

    with pytest.raises(RuntimeError, match="belongs to CAN adapter"):
        DualYamBackend.open(
            robot_id="robot-01",
            left_adapter_serial="WRONG",
            right_adapter_serial="RIGHT",
            calibration_root=tmp_path,
            arm_factory=lambda *_args: pytest.fail("must validate before connecting"),
            interfaces=[_ready("can0", "RIGHT"), _ready("can1", "WRONG")],
        )


def test_partial_open_closes_first_arm(tmp_path):
    _save_calibrations(tmp_path)
    first = FakeArm()
    calls = 0

    def factory(_channel, _limits):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise RuntimeError("right failed")
        return first

    with pytest.raises(RuntimeError, match="right failed"):
        DualYamBackend.open(
            robot_id="robot-01",
            left_adapter_serial="LEFT",
            right_adapter_serial="RIGHT",
            calibration_root=tmp_path,
            arm_factory=factory,
            interfaces=[_ready("can0", "LEFT"), _ready("can1", "RIGHT")],
        )
    assert first.closed is True


def test_arm_only_i2rt_fk_ik_round_trip():
    kinematics = YamKinematics()
    joints = np.array([0.1, 1.2, 1.2, -0.2, 0.1, 0.0])
    pose = kinematics.fk(joints)

    solved = kinematics.ik(pose, joints + 0.01)

    assert kinematics.fk(solved) == pytest.approx(pose, abs=5e-4)


def test_i2rt_arm_rejects_state_when_a_control_worker_has_died():
    dead_thread = SimpleNamespace(name="CAN worker", is_alive=lambda: False)
    live_thread = SimpleNamespace(name="server", is_alive=lambda: True)
    robot = SimpleNamespace(
        _server_thread=live_thread,
        motor_chain=SimpleNamespace(running=True),
    )
    arm = I2RTArm(robot, SimpleNamespace(), (dead_thread,))

    with pytest.raises(RuntimeError, match="CAN worker"):
        arm.read_state()


@dataclass
class MovingArm(FakeArm):
    joints: np.ndarray = field(default_factory=lambda: np.full(6, 0.025))
    gripper: float = 0.95
    apply_commands: bool = True

    def read_state(self):
        return ArmState(self.joints.copy(), self.gripper, np.eye(4), 1.0)

    def command(self, joints, gripper):
        super().command(joints, gripper)
        if self.apply_commands:
            self.joints = joints.copy()
            self.gripper = gripper


def test_move_to_position_bounds_steps_and_verifies_both_arms():
    arms = {"left": MovingArm(), "right": MovingArm(gripper=0.05)}
    backend = DualYamBackend(arms)

    backend.move_to_position(
        DEFAULT_START_POSITION,
        rate_hz=10_000,
        sleep=lambda _delay: None,
        emit=lambda _line: None,
    )

    for side, arm in arms.items():
        initial = np.append(np.full(6, 0.025), 0.95 if side == "left" else 0.05)
        commands = np.asarray([np.append(joints, gripper) for joints, gripper in arm.commands])
        trajectory = np.vstack((initial, commands))
        assert np.max(np.abs(np.diff(trajectory, axis=0))) <= 0.0100001
        assert commands[-1] == pytest.approx(
            np.asarray(DEFAULT_START_POSITION[:7])
            if side == "left"
            else np.asarray(DEFAULT_START_POSITION[7:])
        )
    assert arms["left"].joints == pytest.approx(np.zeros(6))
    assert arms["right"].gripper == pytest.approx(1.0)


def test_move_to_position_timeout_idles_both_arms():
    arms = {
        "left": MovingArm(apply_commands=False),
        "right": MovingArm(apply_commands=False),
    }
    backend = DualYamBackend(arms)
    clock = iter((0.0, 0.0, 0.0, 1.0))

    with pytest.raises(TimeoutError, match="timed out"):
        backend.move_to_position(
            DEFAULT_START_POSITION,
            timeout_seconds=0.5,
            monotonic=lambda: next(clock),
            sleep=lambda _delay: None,
            emit=lambda _line: None,
        )

    assert all(arm.idle_count == 1 for arm in arms.values())


def test_start_position_requires_14_in_limit_values():
    assert validate_dual_position(DEFAULT_START_POSITION).shape == (14,)
    with pytest.raises(ValueError, match="14 finite"):
        validate_dual_position((0.0,) * 13)
    invalid = list(DEFAULT_START_POSITION)
    invalid[1] = -0.1
    with pytest.raises(ValueError, match="left joint 2"):
        validate_dual_position(invalid)

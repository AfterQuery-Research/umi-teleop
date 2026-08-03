from __future__ import annotations

import json
import math

import numpy as np
import pytest

from umi_teleop.can import CANInterfaceInfo
from umi_teleop.controller import (
    GripperMap,
    TeleopLimits,
    TeleopStateMachine,
    decode_packet,
    pose_matrix,
    world_frame_delta,
)
from umi_teleop.yam import (
    DEFAULT_SIM_INITIAL_JOINTS,
    Config,
    SafetyStop,
    find_control_processes,
    gripper_limits_for_mode,
    gripper_position,
    resolve_arm_channels,
    rpy_degrees_to_matrix,
    set_sim_initial_pose,
)


def packet(
    *,
    session="s1",
    seq=0,
    enabled=False,
    zero=0,
    toggle=0,
    cancel=0,
    quit_id=0,
    left_pose=None,
    right_pose=None,
    width=40.0,
):
    ident = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    left_pose = ident if left_pose is None else left_pose
    right_pose = ident if right_pose is None else right_pose

    def arm(pose):
        return {
            "serial": "tracker",
            "pose": pose,
            "pose_age_s": 0.01,
            "width_mm": width,
            "width_age_s": 0.01,
        }

    return {
        "version": 1,
        "session_id": session,
        "seq": seq,
        "requested_enabled": enabled,
        "zero_id": zero,
        "toggle_id": toggle,
        "cancel_id": cancel,
        "quit_id": quit_id,
        "arms": {"left": arm(left_pose), "right": arm(right_pose)},
    }


def machine(**limit_overrides):
    return TeleopStateMachine(
        {"left": GripperMap(20, 80), "right": GripperMap(20, 80)},
        TeleopLimits(**limit_overrides),
    )


ROBOT_POSES = {"left": np.eye(4), "right": np.eye(4)}


def step(m, p, age=0.01):
    return m.step(p, age, ROBOT_POSES)


def test_decoder_accepts_source_schema_and_rejects_nan():
    clean = packet()
    assert decode_packet(json.dumps(clean).encode()) == clean
    clean["arms"]["left"]["width_mm"] = float("nan")
    with pytest.raises(ValueError, match="finite"):
        decode_packet(json.dumps(clean).encode())


def test_process_guard_ignores_own_launch_ancestors_but_not_other_controllers():
    table = """
      100       1 sshd     sshd: yam@notty
      200     100 bash     bash -c python experiments/launch_vive_teleop.py
      300     200 python   python experiments/launch_vive_teleop.py
      400     100 python   python experiments/launch_yaml.py
      500     100 python   python experiments/launch_vive_teleop.py --simulate
    """
    assert find_control_processes(table, own_pid=300) == [
        "400     100 python   python experiments/launch_yaml.py",
        "500     100 python   python experiments/launch_vive_teleop.py --simulate",
    ]


def test_physical_can_channels_follow_adapter_serials_not_kernel_order():
    config = Config(left_adapter_serial="LEFT", right_adapter_serial="RIGHT")
    interfaces = [
        CANInterfaceInfo("can0", "RIGHT", "gs_usb", True, 1_000_000, False),
        CANInterfaceInfo("can1", "LEFT", "gs_usb", True, 1_000_000, False),
    ]

    assert resolve_arm_channels(config, interfaces=interfaces) == {
        "left": "can1",
        "right": "can0",
    }


def test_physical_can_resolution_fails_closed_when_link_is_not_ready():
    config = Config(left_adapter_serial="LEFT", right_adapter_serial="RIGHT")
    interfaces = [
        CANInterfaceInfo("can0", "LEFT", "gs_usb", False, None, False),
        CANInterfaceInfo("can1", "RIGHT", "gs_usb", True, 1_000_000, False),
    ]

    with pytest.raises(SafetyStop, match="umi-setup-can"):
        resolve_arm_channels(config, interfaces=interfaces)


def test_physical_can_requires_distinct_hardware_identities():
    with pytest.raises(SafetyStop, match="left-adapter-serial is required"):
        resolve_arm_channels(Config(), interfaces=[])

    config = Config(left_channel="can0", right_channel="can0")
    with pytest.raises(SafetyStop, match="same SocketCAN interface"):
        resolve_arm_channels(config, interfaces=[])


def test_simulation_uses_default_channels_without_physical_adapters():
    assert resolve_arm_channels(Config(simulate=True)) == {"left": "can0", "right": "can1"}


def test_simulation_uses_normalized_gripper_limits_but_real_uses_raw_endpoints():
    raw = [6.37, 1.22]
    np.testing.assert_allclose(gripper_limits_for_mode(raw, simulate=True), [0.0, 1.0])
    np.testing.assert_allclose(gripper_limits_for_mode(raw, simulate=False), raw)
    with pytest.raises(SafetyStop, match="distinct finite"):
        gripper_limits_for_mode([1.0, 1.0], simulate=False)


def test_gripper_readback_must_be_normalized_before_any_command():
    class FakeRobot:
        def __init__(self, value):
            self.value = value

        def get_observations(self):
            return {"gripper_pos": np.array([self.value])}

    assert gripper_position(FakeRobot(0.4)) == pytest.approx(0.4)
    assert gripper_position(FakeRobot(1.05)) == pytest.approx(1.0)
    with pytest.raises(SafetyStop, match="outside normalized"):
        gripper_position(FakeRobot(1.224))


def test_simulation_starts_at_validated_nonsingular_pose():
    class FakeRobot:
        def __init__(self):
            self.joints = np.zeros(6)
            self.grip = 0.4
            self.commands = []

        def get_observations(self):
            return {
                "joint_pos": self.joints.copy(),
                "gripper_pos": np.array([self.grip]),
            }

        def get_robot_info(self):
            return {"joint_limits": np.array([[-2.0, 2.0]] * 6)}

        def command_joint_pos(self, command):
            self.commands.append(np.asarray(command).copy())
            self.joints = np.asarray(command[:6]).copy()
            self.grip = float(command[6])

    robot = FakeRobot()
    set_sim_initial_pose(robot, list(DEFAULT_SIM_INITIAL_JOINTS), margin=0.03)
    np.testing.assert_allclose(robot.commands[-1][:6], DEFAULT_SIM_INITIAL_JOINTS)
    assert robot.commands[-1][6] == pytest.approx(0.4)

    with pytest.raises(SafetyStop, match="initial joints"):
        set_sim_initial_pose(robot, [2.0, 0, 0, 0, 0, 0], margin=0.03)


def test_new_session_requires_a_new_zero_then_separate_enable():
    m = machine()
    assert "tap ZERO" in step(m, packet(seq=10, zero=4)).reason
    zeroed = step(m, packet(seq=11, zero=5))
    assert zeroed.zero_captured and zeroed.zeroed and not zeroed.armed
    enabled = step(m, packet(seq=12, enabled=True, zero=5, toggle=1))
    assert enabled.armed and enabled.targets is not None


def test_zero_rejected_when_either_pose_stale():
    m = machine()
    step(m, packet())
    p = packet(seq=1, zero=1)
    p["arms"]["right"]["pose_age_s"] = 1.0
    result = step(m, p)
    assert not result.zeroed and "rejected" in result.reason


def test_world_pose_change_maps_from_input_zero_to_robot_zero():
    m = machine(max_translation_m=1.0)
    step(m, packet())
    step(m, packet(seq=1, zero=1))
    moved = [0.1, -0.2, 0.3, 0.0, 0.0, 0.0, 1.0]
    result = step(
        m,
        packet(seq=2, enabled=True, zero=1, toggle=1, left_pose=moved, right_pose=moved),
    )
    np.testing.assert_allclose(result.targets["left"][:3, 3], [0.1, -0.2, 0.3])
    assert result.grippers == {"left": pytest.approx(1 / 3), "right": pytest.approx(1 / 3)}


def test_physical_up_does_not_depend_on_tracker_or_robot_zero_orientation():
    s = math.sqrt(0.5)
    input_zero = pose_matrix([1, 2, 3, 0, 0, s, s])  # tracker yawed +90 deg
    input_up = pose_matrix([1, 2, 3.05, 0, 0, s, s])
    delta = world_frame_delta(input_zero, input_up)
    np.testing.assert_allclose(delta[:3, 3], [0.0, 0.0, 0.05], atol=1e-9)
    np.testing.assert_allclose(delta[:3, :3], np.eye(3), atol=1e-9)

    robot_zero = np.eye(4)
    robot_zero[:3, :3] = rpy_degrees_to_matrix([40, -20, 70])
    m = machine(max_translation_m=1.0)
    first = packet(left_pose=[1, 2, 3, 0, 0, s, s])
    m.step(first, 0.01, {"left": robot_zero, "right": np.eye(4)})
    zeroed = packet(seq=1, zero=1, left_pose=[1, 2, 3, 0, 0, s, s])
    m.step(zeroed, 0.01, {"left": robot_zero, "right": np.eye(4)})
    moved = packet(
        seq=2,
        enabled=True,
        zero=1,
        toggle=1,
        left_pose=[1, 2, 3.05, 0, 0, s, s],
    )
    result = m.step(moved, 0.01, {"left": robot_zero, "right": np.eye(4)})
    np.testing.assert_allclose(
        result.targets["left"][:3, 3], robot_zero[:3, 3] + [0, 0, 0.05], atol=1e-9
    )
    np.testing.assert_allclose(result.targets["left"][:3, :3], robot_zero[:3, :3], atol=1e-9)


def test_fixed_basis_maps_world_translation_and_rotation_about_robot_base_axes():
    basis = rpy_degrees_to_matrix([0, 0, 90])
    zero = np.eye(4)
    current = np.eye(4)
    current[:3, 3] = [0.1, 0.0, 0.0]
    current[:3, :3] = rpy_degrees_to_matrix([20, 0, 0])
    delta = world_frame_delta(zero, current, basis)
    np.testing.assert_allclose(delta[:3, 3], [0.0, 0.1, 0.0], atol=1e-9)
    np.testing.assert_allclose(delta[:3, :3], rpy_degrees_to_matrix([0, 20, 0]), atol=1e-9)


def test_tip_offset_accounts_for_rotation_pivot():
    m = TeleopStateMachine(
        {"left": GripperMap(0, 1), "right": GripperMap(0, 1)},
        TeleopLimits(max_translation_m=2.0),
        tip_offsets_m={"left": np.array([1.0, 0.0, 0.0]), "right": np.zeros(3)},
    )
    step(m, packet(width=0.5))
    step(m, packet(seq=1, zero=1, width=0.5))
    s = math.sqrt(0.5)
    rotated = [0.0, 0.0, 0.0, 0.0, 0.0, s, s]
    result = step(
        m,
        packet(
            seq=2,
            enabled=True,
            zero=1,
            toggle=1,
            left_pose=rotated,
            width=0.5,
        ),
    )
    np.testing.assert_allclose(result.targets["left"][:3, 3], [-1.0, 1.0, 0.0], atol=1e-6)


def test_stale_packet_or_pose_disarms_without_automatic_rearm():
    m = machine()
    step(m, packet())
    step(m, packet(seq=1, zero=1))
    assert step(m, packet(seq=2, enabled=True, zero=1, toggle=1)).armed
    assert not step(m, packet(seq=3, enabled=True, zero=1, toggle=1), age=1.0).armed
    # Fresh data with the same toggle counter cannot silently re-arm.
    assert not step(m, packet(seq=4, enabled=True, zero=1, toggle=1)).armed
    assert step(m, packet(seq=5, enabled=True, zero=1, toggle=2)).armed


def test_session_change_and_cancel_disarm():
    m = machine()
    step(m, packet())
    step(m, packet(seq=1, zero=1))
    step(m, packet(seq=2, enabled=True, zero=1, toggle=1))
    assert not step(m, packet(seq=3, cancel=1, zero=1, toggle=1)).armed
    result = step(m, packet(session="s2", seq=500, enabled=True, zero=99, toggle=99))
    assert not result.armed and not result.zeroed


def test_motion_envelope_disarms_before_emitting_target():
    m = machine(max_translation_m=0.05, max_rotation_rad=math.radians(10))
    step(m, packet())
    step(m, packet(seq=1, zero=1))
    result = step(
        m,
        packet(
            seq=2,
            enabled=True,
            zero=1,
            toggle=1,
            left_pose=[0.1, 0, 0, 0, 0, 0, 1],
        ),
    )
    assert not result.armed and result.targets is None and "translation" in result.reason


def test_pose_matrix_rotates_tip_offset():
    s = math.sqrt(0.5)
    pose = pose_matrix([1, 2, 3, 0, 0, s, s], np.array([1, 0, 0]))
    np.testing.assert_allclose(pose[:3, 3], [1, 3, 3], atol=1e-6)

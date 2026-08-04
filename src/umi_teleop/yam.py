#!/usr/bin/env python3
"""Safely teleoperate both YAM arms from the dual-UMI telemetry source."""

from __future__ import annotations

import fcntl
import inspect
import logging
import math
import os
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Literal

import numpy as np
import tyro

from umi_teleop.can import (
    YAM_CAN_BITRATE,
    CANInterfaceInfo,
    can_interface_readiness_error,
    discover_can_interfaces,
    resolve_can_interface,
)
from umi_teleop.controller import (
    ARM_NAMES,
    GripperMap,
    TeleopLimits,
    TeleopStateMachine,
    UdpTeleopReceiver,
    pose_matrix,
    rotation_angle,
    world_frame_delta,
)
from umi_teleop.dataset import (
    ControlLagMonitor,
    DatasetConfig,
    DatasetFrameError,
    DatasetRecorder,
    DatasetRecordingError,
    commit_dataset_step,
    create_dataset_recorder,
    prepare_dataset_step,
)
from umi_teleop.ik_policy import (
    AsyncFallbackIkSolver,
    FallbackIkSolver,
    IkFallbackResult,
    fallback_diagnostic,
)

KNOWN_CONTROL_PATTERNS = (
    "umi-yam",
    "umi_teleop.yam",
    "launch_yaml_eval_molmoact.py",
    "launch_yaml.py",
    "safe_yam_control.py",
    "control_with_viser.py",
    "yam_zero_gravity",
    "zero_gravity_hold",
    "host_server_yam.py",
    "rd teleop",
    "rd record",
    "rd replay",
    "rd serve",
)
LOCK_PREFIX = "/tmp/yam_vive_teleop"
DEFAULT_SIM_INITIAL_JOINTS = (0.0, 0.8, 1.2, 0.0, 0.0, 0.0)
DEFAULT_READY_BEND_JOINTS = (0.4, 0.8)
DEFAULT_READY_JOINTS = (0.0, 0.4, 0.8, 0.0, 0.0, 0.0)
DEFAULT_BASE_JOINTS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
DEFAULT_STARTUP_READBACK_TOLERANCE = 0.06


@dataclass(frozen=True)
class Config:
    """YAM receiver, startup motion, and teleoperation safety settings."""

    bind: str = "0.0.0.0"
    port: int = 9140
    left_adapter_serial: str | None = None
    right_adapter_serial: str | None = None
    left_channel: str | None = None
    right_channel: str | None = None
    left_gripper_limits: tuple[float, float] = (6.3704509, 1.2239643)
    right_gripper_limits: tuple[float, float] = (6.3967727, 1.2010758)
    left_input_width_mm: tuple[float, float] = (40.376321, 122.999636)
    right_input_width_mm: tuple[float, float] = (40.128507, 122.685643)
    left_tip_offset: tuple[float, float, float] = (-0.08370, 0.0, 0.21395247527)
    right_tip_offset: tuple[float, float, float] = (-0.08370, 0.0, 0.21395247527)
    vive_to_robot_rpy_deg: tuple[float, float, float] = (0.0, 0.0, 0.0)
    zero_frame_source: Literal["fixed", "left", "right", "each"] = "right"
    zero_frame_axis_map: Literal["identity", "umi-handle"] = "umi-handle"
    hz: float = 30.0
    max_packet_age: float = 1.0
    max_pose_age: float = 0.25
    max_width_age: float = 0.75
    max_translation: float = 0.50
    max_rotation_deg: float = 90.0
    position_scale: float = 1.50
    max_joint_speed: float = 0.45
    max_gripper_speed: float = 1.50
    joint_limit_margin: float = 0.03
    move_to_ready: bool = False
    move_to_base: bool = True
    ready_bend_joints: tuple[float, float] = DEFAULT_READY_BEND_JOINTS
    ready_joint_speed: float = 0.10
    base_joint_speed: float = 0.10
    startup_readback_tolerance: float = DEFAULT_STARTUP_READBACK_TOLERANCE
    ik_max_iters: int = 40
    ik_pos_threshold: float = 1e-3
    ik_ori_threshold: float = 2e-3
    ik_fallback_attempts_per_cycle: int = 2
    ik_hold_disarm_s: float = 0.0
    ik_result_max_age_s: float = 0.25
    stream_timeout: float = 20.0
    monitor_only: bool = False
    simulate: bool = False
    sim_initial_joints: tuple[float, float, float, float, float, float] = DEFAULT_SIM_INITIAL_JOINTS
    confirm_clear: bool = False
    dataset: DatasetConfig = field(default_factory=DatasetConfig)


class SafetyStop(RuntimeError):
    pass


class StartupReadbackError(SafetyStop):
    def __init__(self, message: str, observed: dict[str, np.ndarray]):
        super().__init__(message)
        self.observed = {name: values.copy() for name, values in observed.items()}


def find_control_processes(process_table: str, own_pid: int) -> list[str]:
    """Return controller rows, excluding this process and its launch ancestors."""
    rows: list[tuple[int, int, str, str, str]] = []
    parents: dict[int, int] = {}
    for line in process_table.splitlines():
        parts = line.strip().split(None, 3)
        if len(parts) != 4:
            continue
        pid_text, ppid_text, command, arguments = parts
        try:
            pid = int(pid_text)
            ppid = int(ppid_text)
        except ValueError:
            continue
        parents[pid] = ppid
        rows.append((pid, ppid, command, arguments, line.strip()))

    ignored = {own_pid}
    cursor = own_pid
    while (parent := parents.get(cursor)) is not None and parent > 1 and parent not in ignored:
        ignored.add(parent)
        cursor = parent

    matches = []
    for pid, _ppid, _command, arguments, display in rows:
        if pid in ignored:
            continue
        if "launch_vive_teleop.py" in arguments or any(
            pattern in arguments for pattern in KNOWN_CONTROL_PATTERNS
        ):
            matches.append(display)
    return matches


def check_no_other_control_processes() -> None:
    proc = subprocess.run(
        ["ps", "-eo", "pid=,ppid=,comm=,args="],
        check=True,
        text=True,
        capture_output=True,
    )
    matches = find_control_processes(proc.stdout, os.getpid())
    if matches:
        raise SafetyStop("another robot-control process is running:\n  " + "\n  ".join(matches))


def resolve_arm_channels(
    args: Config,
    *,
    interfaces: list[CANInterfaceInfo] | None = None,
) -> dict[str, str]:
    if args.simulate:
        if args.left_adapter_serial is not None or args.right_adapter_serial is not None:
            raise SafetyStop("CAN adapter serials must not be set in simulation")
        resolved = {
            "left": args.left_channel or "can0",
            "right": args.right_channel or "can1",
        }
    else:
        arm_values = {
            "left": (args.left_adapter_serial, args.left_channel),
            "right": (args.right_adapter_serial, args.right_channel),
        }
        discovered = (
            discover_can_interfaces()
            if interfaces is None and any(serial is not None for serial, _ in arm_values.values())
            else interfaces
        )
        resolved = {}
        readiness_errors = []
        for side, (serial, channel) in arm_values.items():
            if serial is not None:
                serial = serial.strip()
                if not serial:
                    raise SafetyStop(f"--{side}-adapter-serial must not be empty")
            if channel is not None:
                channel = channel.strip()
                if not channel:
                    raise SafetyStop(f"--{side}-channel must not be empty")
            if serial is not None and channel is not None:
                raise SafetyStop(
                    f"--{side}-adapter-serial and --{side}-channel are mutually exclusive"
                )
            if serial is None and channel is None:
                raise SafetyStop(
                    f"--{side}-adapter-serial is required for a physical YAM arm "
                    f"(or set --{side}-channel for an externally managed interface)"
                )
            if serial is None:
                resolved[side] = channel
                continue

            try:
                interface = resolve_can_interface(serial, interfaces=discovered)
            except (RuntimeError, ValueError) as exc:
                raise SafetyStop(str(exc)) from exc
            readiness_error = can_interface_readiness_error(interface)
            if readiness_error is not None:
                readiness_errors.append(
                    f"{side} adapter {serial} resolved to {interface.name}: {readiness_error}"
                )
            logging.getLogger(__name__).info(
                "Resolved %s YAM CAN adapter %s to %s", side, serial, interface.name
            )
            resolved[side] = interface.name

        if readiness_errors:
            setup_flags = " ".join(
                f"--{side}-adapter-serial {serial}"
                for side, (serial, _channel) in arm_values.items()
                if serial is not None
            )
            raise SafetyStop(
                "SocketCAN is not ready:\n  "
                + "\n  ".join(readiness_errors)
                + f"\nRun `umi-setup-can {setup_flags}` before starting umi-yam."
            )

    if resolved["left"] == resolved["right"]:
        raise SafetyStop("left and right YAM arms resolved to the same SocketCAN interface")
    return resolved


def check_can_interface(channel: str) -> None:
    if not (Path("/sys/class/net") / channel).exists():
        raise SafetyStop(f"CAN interface {channel!r} does not exist")
    try:
        interface = next(
            (item for item in discover_can_interfaces() if item.name == channel),
            None,
        )
    except RuntimeError as exc:
        raise SafetyStop(str(exc)) from exc
    if interface is None:
        raise SafetyStop(f"CAN interface {channel!r} is not a SocketCAN interface")
    readiness_error = can_interface_readiness_error(
        interface,
        bitrate=YAM_CAN_BITRATE,
        use_fd=False,
    )
    if readiness_error is not None:
        raise SafetyStop(f"CAN interface {channel!r} is not ready: {readiness_error}")
    proc = subprocess.run(
        ["ip", "-details", "-statistics", "link", "show", channel],
        check=True,
        text=True,
        capture_output=True,
    )
    output = proc.stdout
    if "state ERROR-ACTIVE" not in output or "UP" not in output.split(">", 1)[0]:
        raise SafetyStop(f"CAN interface {channel!r} is not UP and ERROR-ACTIVE")
    counter_error = can_error_counter_error(output)
    if counter_error is not None:
        raise SafetyStop(f"CAN interface {channel!r} {counter_error}")


def can_error_counter_error(output: str) -> str | None:
    """Return a current CAN fault while tolerating recovered cumulative transitions."""
    match = re.search(
        r"re-started\s+bus-errors\s+arbit-lost\s+error-warn\s+error-pass\s+bus-off\s*\n"
        r"\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)",
        output,
    )
    if match is None:
        return "has unreadable error counters"
    _restarted, bus_errors, _arbitration_lost, _error_warn, _error_pass, bus_off = (
        int(value) for value in match.groups()
    )
    # warning/passive are cumulative transition counters. They may remain nonzero after the bus
    # has recovered to ERROR-ACTIVE, so treating them as current faults prevents safe reuse until
    # a privileged interface reset. Actual bus errors and bus-off events remain fail-closed.
    if bus_errors or bus_off:
        return f"has bus-errors={bus_errors}, bus-off={bus_off}"
    return None


def acquire_locks(channels: tuple[str, str]) -> list[Any]:
    locks = []
    try:
        for channel in channels:
            safe = re.sub(r"[^A-Za-z0-9_.-]", "_", channel)
            lock = open(f"{LOCK_PREFIX}.{safe}.lock", "w", encoding="utf-8")
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            lock.write(f"{os.getpid()}\n")
            lock.flush()
            locks.append(lock)
    except BlockingIOError as exc:
        lock.close()
        for held in locks:
            held.close()
        raise SafetyStop(f"another Vive controller holds the {channel} lock") from exc
    return locks


def close_robot(robot: Any) -> None:
    """Stop I2RT's command thread before closing CAN (avoids its shutdown race)."""
    stop_event = getattr(robot, "_stop_event", None)
    server_thread = getattr(robot, "_server_thread", None)
    motor_chain = getattr(robot, "motor_chain", None)
    if stop_event is not None:
        stop_event.set()
    if server_thread is not None:
        server_thread.join(timeout=1.0)
    if motor_chain is not None:
        motor_chain.running = False
        time.sleep(0.08)
        motor_chain.close()
    else:
        robot.close()


def arm_joints(robot: Any) -> np.ndarray:
    obs = robot.get_observations()
    return np.asarray(obs["joint_pos"], dtype=np.float64).copy()


def _gripper_position_from_observation(obs: dict[str, Any]) -> float:
    value = float(np.asarray(obs["gripper_pos"]).ravel()[0])
    if not math.isfinite(value) or value < -0.1 or value > 1.1:
        raise SafetyStop(
            f"gripper readback {value!r} is outside normalized command space; "
            "verify I2RT gripper limits before commanding"
        )
    return float(np.clip(value, 0.0, 1.0))


def gripper_position(robot: Any) -> float:
    return _gripper_position_from_observation(robot.get_observations())


def read_pair_state(
    robots: dict[str, Any],
) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    """Read joints and gripper together, acquiring each I2RT state lock once."""
    joints: dict[str, np.ndarray] = {}
    grippers: dict[str, float] = {}
    for name in ARM_NAMES:
        observation = robots[name].get_observations()
        joints[name] = np.asarray(observation["joint_pos"], dtype=np.float64).copy()
        grippers[name] = _gripper_position_from_observation(observation)
    return joints, grippers


def gripper_limits_for_mode(limits: list[float], simulate: bool) -> np.ndarray:
    """Use normalized limits in SimRobot; raw endpoints are only for real MotorChainRobot."""
    raw = np.asarray(limits, dtype=np.float64)
    if raw.shape != (2,) or not np.isfinite(raw).all() or abs(raw[1] - raw[0]) < 1e-6:
        raise SafetyStop("gripper limits must contain two distinct finite endpoints")
    return np.array([0.0, 1.0], dtype=np.float64) if simulate else raw


def rpy_degrees_to_matrix(rpy_degrees: list[float] | tuple[float, float, float]) -> np.ndarray:
    """Build ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` for Vive-world to robot-base axes."""
    rpy = np.asarray(rpy_degrees, dtype=np.float64)
    if rpy.shape != (3,) or not np.isfinite(rpy).all():
        raise SafetyStop("--vive-to-robot-rpy-deg must contain three finite angles")
    roll, pitch, yaw = np.radians(rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


class IkModel:
    def __init__(self, max_iters: int, pos_threshold: float, ori_threshold: float):
        from i2rt.robots.kinematics import Kinematics
        from i2rt.robots.utils import ArmType, GripperType, combine_arm_and_gripper_xml

        xml = combine_arm_and_gripper_xml(ArmType.YAM, GripperType.NO_GRIPPER)
        self.model = Kinematics(xml, "grasp_site")
        self.max_iters = max_iters
        self.pos_threshold = pos_threshold
        self.ori_threshold = ori_threshold

    def fk(self, joints: np.ndarray) -> np.ndarray:
        return np.asarray(self.model.fk(joints, "grasp_site"), dtype=np.float64)

    def solve(self, target: np.ndarray, initial: np.ndarray) -> tuple[bool, np.ndarray]:
        success, joints = self.model.ik(
            target,
            "grasp_site",
            init_q=initial,
            max_iters=self.max_iters,
            pos_threshold=self.pos_threshold,
            ori_threshold=self.ori_threshold,
        )
        return bool(success), np.asarray(joints, dtype=np.float64).copy()


def make_robot(channel: str, limits: list[float], simulate: bool) -> Any:
    from i2rt.robots.utils import ArmType, GripperType

    from umi_teleop.i2rt_adapter import get_yam_robot

    required = {"arm_type", "gripper_limits_override", "sim", "enable_auto_recovery"}
    available = set(inspect.signature(get_yam_robot).parameters)
    if not required <= available:
        raise SafetyStop("I2RT >= 1.2.4 is required for guarded gripper limits and simulation")
    return get_yam_robot(
        channel=channel,
        arm_type=ArmType.YAM,
        gripper_type=GripperType.LINEAR_4310,
        zero_gravity_mode=False,
        gripper_limits_override=gripper_limits_for_mode(limits, simulate),
        enable_auto_recovery=False,
        sim=simulate,
    )


def wait_for_stream(
    receiver: UdpTeleopReceiver,
    timeout_s: float,
    max_packet_age_s: float,
    max_pose_age_s: float,
    max_width_age_s: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        packet, age = receiver.poll()
        if packet is not None and age is not None and age <= max_packet_age_s:
            if all(
                packet["arms"][name]["pose"] is not None
                and packet["arms"][name]["pose_age_s"] <= max_pose_age_s
                and packet["arms"][name]["width_mm"] is not None
                and packet["arms"][name]["width_age_s"] <= max_width_age_s
                for name in ARM_NAMES
            ):
                return packet
        time.sleep(0.02)
    raise SafetyStop(
        "timed out waiting for two fresh Vive trackers and ArUco widths; no motors were connected"
    )


def check_joint_target(robot: Any, target: np.ndarray, margin: float) -> bool:
    return joint_limit_reason(robot, target, margin) is None


def joint_limit_reason(robot: Any, target: np.ndarray, margin: float) -> str | None:
    """Describe the first joint that is invalid or inside the configured limit margin."""
    limits = np.asarray(robot.get_robot_info()["joint_limits"], dtype=np.float64)
    return joint_limit_reason_from_limits(limits, target, margin)


def joint_limit_reason_from_limits(
    limits: np.ndarray, target: np.ndarray, margin: float
) -> str | None:
    """Validate joints against a cached limit table without touching a hardware object."""
    limits = np.asarray(limits, dtype=np.float64)
    if target.shape != (6,) or not np.isfinite(target).all() or limits.shape != (6, 2):
        return "has invalid joint data"
    for index, (value, bounds) in enumerate(zip(target, limits, strict=True), start=1):
        low, high = bounds
        if value < low + margin:
            return (
                f"joint {index} is {math.degrees(value):.1f}deg, below the "
                f"{math.degrees(low + margin):.1f}deg safety margin"
            )
        if value > high - margin:
            return (
                f"joint {index} is {math.degrees(value):.1f}deg, above the "
                f"{math.degrees(high - margin):.1f}deg safety margin"
            )
    return None


def ready_joint_target(current: np.ndarray, bend_joints: list[float]) -> np.ndarray:
    """Keep base/wrist joints fixed while bending shoulder and elbow for Cartesian IK."""
    target = np.asarray(current, dtype=np.float64).copy()
    bend = np.asarray(bend_joints, dtype=np.float64)
    if (
        target.shape != (6,)
        or bend.shape != (2,)
        or not np.isfinite(target).all()
        or not np.isfinite(bend).all()
    ):
        raise SafetyStop("teleop-ready joints must be finite with shapes (6,) and (2,)")
    target[1:3] = bend
    return target


def move_pair_to_ready(
    robots: dict[str, Any],
    joints: dict[str, np.ndarray],
    grips: dict[str, float],
    bend_joints: list[float],
    speed_rad_s: float,
    hz: float,
    margin: float,
    readback_tolerance: float = DEFAULT_STARTUP_READBACK_TOLERANCE,
    recorder: DatasetRecorder | None = None,
) -> dict[str, np.ndarray]:
    """Move both arms through a safe bend to the canonical teleoperation pose."""
    bend_targets = {name: ready_joint_target(joints[name], bend_joints) for name in ARM_NAMES}
    bent = move_pair_to_joint_targets(
        robots,
        joints,
        grips,
        bend_targets,
        speed_rad_s,
        hz,
        margin,
        readback_tolerance,
        label=(
            "teleop-ready bend "
            f"J2={math.degrees(float(bend_joints[0])):.1f}deg "
            f"J3={math.degrees(float(bend_joints[1])):.1f}deg"
        ),
        recorder=recorder,
    )
    canonical_targets = {
        name: np.asarray(DEFAULT_READY_JOINTS, dtype=np.float64).copy() for name in ARM_NAMES
    }
    return move_pair_to_joint_targets(
        robots,
        bent,
        grips,
        canonical_targets,
        speed_rad_s,
        hz,
        margin,
        readback_tolerance,
        label="canonical teleop-ready pose [0, 23, 46, 0, 0, 0]deg",
        recorder=recorder,
    )


def move_pair_to_base(
    robots: dict[str, Any],
    joints: dict[str, np.ndarray],
    grips: dict[str, float],
    speed_rad_s: float,
    hz: float,
    readback_tolerance: float = DEFAULT_STARTUP_READBACK_TOLERANCE,
) -> dict[str, np.ndarray]:
    """Slowly park both arms at the repository-defined all-zero base pose."""
    targets = {name: np.asarray(DEFAULT_BASE_JOINTS, dtype=np.float64).copy() for name in ARM_NAMES}
    return move_pair_to_joint_targets(
        robots,
        joints,
        grips,
        targets,
        speed_rad_s,
        hz,
        margin=0.0,
        readback_tolerance=readback_tolerance,
        label="base pose J1-J6=0deg",
    )


def move_pair_to_joint_targets(
    robots: dict[str, Any],
    joints: dict[str, np.ndarray],
    grips: dict[str, float],
    targets: dict[str, np.ndarray],
    speed_rad_s: float,
    hz: float,
    margin: float,
    readback_tolerance: float,
    label: str,
    recorder: DatasetRecorder | None = None,
) -> dict[str, np.ndarray]:
    """Linearly interpolate both arms to validated joint targets."""
    if speed_rad_s <= 0.0 or hz <= 0.0 or readback_tolerance <= 0.0:
        raise SafetyStop(
            "joint-motion speed, control rate, and readback tolerance must be positive"
        )
    if set(targets) != set(ARM_NAMES):
        raise SafetyStop("joint targets must contain left and right")
    for name in ARM_NAMES:
        reason = joint_limit_reason(robots[name], targets[name], margin)
        if reason is not None:
            raise SafetyStop(f"{name} {label} target {reason}")
    max_delta = max(float(np.max(np.abs(targets[name] - joints[name]))) for name in ARM_NAMES)
    steps = max(1, math.ceil(max_delta / (speed_rad_s / hz)))
    starts = {name: joints[name].copy() for name in ARM_NAMES}
    print(
        f"moving both YAMs to {label} at {math.degrees(speed_rad_s):.1f}deg/s",
        flush=True,
    )
    period = 1.0 / hz
    next_step = time.monotonic()
    for step_index in range(1, steps + 1):
        alpha = step_index / steps
        commanded = {
            name: starts[name] + alpha * (targets[name] - starts[name]) for name in ARM_NAMES
        }
        command_startup_pair(robots, commanded, grips, recorder)
        next_step += period
        time.sleep(max(0.0, next_step - time.monotonic()))

    deadline = time.monotonic() + 3.0
    observed: dict[str, np.ndarray] = {}
    while True:
        command_startup_pair(robots, targets, grips, recorder)
        observed, _ = read_pair_state(robots)
        max_error = max(float(np.max(np.abs(observed[name] - targets[name]))) for name in ARM_NAMES)
        if max_error <= readback_tolerance:
            break
        if time.monotonic() >= deadline:
            raise StartupReadbackError(
                f"{label} failed readback verification; max joint error "
                f"{math.degrees(max_error):.1f}deg",
                observed,
            )
        next_step += period
        time.sleep(max(0.0, next_step - time.monotonic()))
    for name in ARM_NAMES:
        values = ",".join(f"{value:+.1f}" for value in np.degrees(observed[name]))
        print(f"{label} verified {name} joints_deg=[{values}]", flush=True)
    return {name: observed[name].copy() for name in ARM_NAMES}


def set_sim_initial_pose(robot: Any, target: list[float], margin: float) -> None:
    """Put SimRobot at an explicit, nonsingular pose before zeroing."""
    joints = np.asarray(target, dtype=np.float64)
    if not check_joint_target(robot, joints, margin):
        raise SafetyStop("simulation initial joints are invalid or too close to a joint limit")
    grip = gripper_position(robot)
    robot.command_joint_pos(np.concatenate([joints, [grip]]))
    observed = arm_joints(robot)
    if observed.shape != (6,) or not np.allclose(observed, joints, atol=1e-6, rtol=0.0):
        raise SafetyStop("simulation failed to enter the requested initial joint pose")


def command_pair(
    robots: dict[str, Any], joints: dict[str, np.ndarray], grips: dict[str, float]
) -> None:
    for name in ARM_NAMES:
        motor_chain = getattr(robots[name], "motor_chain", None)
        if motor_chain is not None and not getattr(motor_chain, "running", True):
            raise SafetyStop(f"{name} motor chain stopped; refusing further commands")
        server_thread = getattr(robots[name], "_server_thread", None)
        if server_thread is not None and not server_thread.is_alive():
            raise SafetyStop(f"{name} robot command thread stopped")
        robots[name].command_joint_pos(np.concatenate([joints[name], [grips[name]]]))


def command_startup_pair(
    robots: dict[str, Any],
    joints: dict[str, np.ndarray],
    grips: dict[str, float],
    recorder: DatasetRecorder | None,
) -> None:
    """Capture a startup observation, dispatch both arms, then commit its action."""
    prepared_frame = None
    if recorder is not None:
        measured_joints, measured_grippers = read_pair_state(robots)
        prepared_frame = recorder.prepare_frame(
            measured_joints=measured_joints,
            measured_grippers=measured_grippers,
            commanded_joints=joints,
            commanded_grippers=grips,
        )
    command_pair(robots, joints, grips)
    if recorder is not None:
        assert prepared_frame is not None
        recorder.commit_frame(prepared_frame)


def monitor_only(receiver: UdpTeleopReceiver) -> None:
    print(
        "monitoring UMI source only; no motor imports/connections (Ctrl+C to stop)\n"
        "dxyz_ros_mm uses fixed ROS axes: +X forward, +Y left, +Z up",
        flush=True,
    )
    last = 0.0
    reference: dict[str, np.ndarray] = {}
    reference_zero_id: int | None = None
    while True:
        packet, age = receiver.poll()
        now = time.monotonic()
        if packet is not None and now - last >= 0.5:
            if reference_zero_id != packet["zero_id"]:
                reference.clear()
                reference_zero_id = packet["zero_id"]
            arm_text = []
            for name in ARM_NAMES:
                arm = packet["arms"][name]
                pose = None if arm["pose"] is None else pose_matrix(arm["pose"])
                if pose is not None and name not in reference:
                    reference[name] = pose.copy()
                if pose is None or name not in reference:
                    delta_text = "dxyz_ros_mm=unavailable drot_deg=unavailable"
                else:
                    delta = world_frame_delta(reference[name], pose)
                    dxyz_mm = 1000.0 * delta[:3, 3]
                    delta_text = (
                        "dxyz_ros_mm="
                        f"[{dxyz_mm[0]:+.1f},{dxyz_mm[1]:+.1f},{dxyz_mm[2]:+.1f}] "
                        f"drot_deg={math.degrees(rotation_angle(delta[:3, :3])):.1f}"
                    )
                arm_text.append(
                    f"{name}:pose_age={arm['pose_age_s']} width={arm['width_mm']} "
                    f"width_age={arm['width_age_s']} {delta_text}"
                )
            print(
                f"session={packet['session_id'][:8]} seq={packet['seq']} packet_age={age:.3f} "
                f"enabled={packet['requested_enabled']} zero={packet['zero_id']} "
                + " | ".join(arm_text),
                flush=True,
            )
            last = now
        time.sleep(0.01)


def run(args: Config) -> None:
    if args.move_to_ready and args.move_to_base:
        raise SafetyStop("move-to-ready and move-to-base are mutually exclusive")
    if args.monitor_only and args.dataset.enabled:
        raise SafetyStop("dataset recording cannot be combined with monitor-only mode")
    if args.ik_fallback_attempts_per_cycle < 1:
        raise SafetyStop("--ik-fallback-attempts-per-cycle must be at least one")
    if args.ik_hold_disarm_s < 0.0:
        raise SafetyStop("--ik-hold-disarm-s must be nonnegative")
    if args.ik_result_max_age_s <= 0.0:
        raise SafetyStop("--ik-result-max-age-s must be positive")
    if not args.monitor_only and not args.simulate and not args.confirm_clear:
        raise SafetyStop(
            "this command can move physical arms; clear the workspace and pass --confirm-clear"
        )
    receiver = UdpTeleopReceiver(args.bind, args.port)
    if args.monitor_only:
        try:
            monitor_only(receiver)
        finally:
            receiver.close()
        return

    for config_field in (
        "left_gripper_limits",
        "right_gripper_limits",
        "left_input_width_mm",
        "right_input_width_mm",
    ):
        if getattr(args, config_field) is None:
            raise SafetyStop(f"--{config_field.replace('_', '-')} is required for motor control")

    limits = TeleopLimits(
        max_packet_age_s=args.max_packet_age,
        max_pose_age_s=args.max_pose_age,
        max_width_age_s=args.max_width_age,
        max_translation_m=args.max_translation,
        max_rotation_rad=math.radians(args.max_rotation_deg),
        position_scale=args.position_scale,
        require_width_to_arm=True,
    )
    state = TeleopStateMachine(
        {
            "left": GripperMap(*args.left_input_width_mm),
            "right": GripperMap(*args.right_input_width_mm),
        },
        limits=limits,
        tip_offsets_m={
            "left": np.asarray(args.left_tip_offset, dtype=np.float64),
            "right": np.asarray(args.right_tip_offset, dtype=np.float64),
        },
        input_to_robot_rotation=rpy_degrees_to_matrix(args.vive_to_robot_rpy_deg),
        zero_frame_source=args.zero_frame_source,
        zero_frame_axis_map=args.zero_frame_axis_map,
    )
    print(
        "Vive world -> robot base RPY(deg)="
        f"{np.asarray(args.vive_to_robot_rpy_deg, dtype=np.float64)}; "
        f"zero_frame_source={args.zero_frame_source}; "
        f"zero_frame_axis_map={args.zero_frame_axis_map}; "
        "fixed ROS +X=forward +Y=left +Z=up before ZERO-frame correction",
        flush=True,
    )

    # Mink/MuJoCo allocation can hold the GIL; do it before any motor command thread exists.
    fk_models = {
        name: IkModel(args.ik_max_iters, args.ik_pos_threshold, args.ik_ori_threshold)
        for name in ARM_NAMES
    }
    solve_models = {
        name: IkModel(args.ik_max_iters, args.ik_pos_threshold, args.ik_ori_threshold)
        for name in ARM_NAMES
    }
    initial_packet = wait_for_stream(
        receiver,
        args.stream_timeout,
        args.max_packet_age,
        args.max_pose_age,
        args.max_width_age,
    )
    if (args.move_to_ready or args.move_to_base) and initial_packet["requested_enabled"]:
        raise SafetyStop(
            "UMI source must be disabled before an automatic startup motion; hold CANCEL"
        )

    channels: tuple[str, ...] = ()
    locks: list[Any] = []
    robots: dict[str, Any] = {}
    async_ik: dict[str, AsyncFallbackIkSolver] = {}
    recorder = None
    try:
        try:
            recorder = create_dataset_recorder(args.dataset, args.hz)
        except DatasetRecordingError as exc:
            raise SafetyStop(str(exc)) from exc
        if recorder is not None:
            print(
                f"DATASET ready repo={args.dataset.repo_id} root={args.dataset.root} "
                f"resume={args.dataset.resume} upload={args.dataset.push_to_hub} "
                f"pipeline={'streaming' if args.dataset.streaming_encoding else 'png-staging'} "
                f"codec={args.dataset.video_codec} gop={args.dataset.encoder_gop_size}",
                flush=True,
            )

        channels_by_side = resolve_arm_channels(args)
        channels = tuple(channels_by_side[name] for name in ARM_NAMES)
        if not args.simulate:
            check_no_other_control_processes()
            for channel in channels:
                check_can_interface(channel)
            locks = acquire_locks(channels)
        robots["left"] = make_robot(
            channels_by_side["left"], args.left_gripper_limits, args.simulate
        )
        robots["right"] = make_robot(
            channels_by_side["right"], args.right_gripper_limits, args.simulate
        )

        if args.simulate:
            for robot in robots.values():
                set_sim_initial_pose(robot, args.sim_initial_joints, args.joint_limit_margin)
            print(
                "simulation initialized at nonsingular joints "
                f"{np.asarray(args.sim_initial_joints)}",
                flush=True,
            )
        joints, grips = read_pair_state(robots)
        base_pose_verified = False
        if args.move_to_base:
            try:
                joints = move_pair_to_base(
                    robots,
                    joints,
                    grips,
                    args.base_joint_speed,
                    args.hz,
                    args.startup_readback_tolerance,
                )
                base_pose_verified = True
                startup_blocked = None
            except StartupReadbackError as exc:
                joints = exc.observed
                startup_blocked = str(exc)
        elif args.move_to_ready:
            try:
                joints = move_pair_to_ready(
                    robots,
                    joints,
                    grips,
                    args.ready_bend_joints,
                    args.ready_joint_speed,
                    args.hz,
                    args.joint_limit_margin,
                    args.startup_readback_tolerance,
                )
                startup_blocked = None
            except StartupReadbackError as exc:
                joints = exc.observed
                startup_blocked = str(exc)
        else:
            startup_blocked = None
        command_pair(robots, joints, grips)
        if startup_blocked is None:
            print("robots holding current pose; tap the Pi ZERO button before enable", flush=True)
        else:
            print(
                "STARTUP HOLD: reset motion did not fully verify, so teleop enable is locked out "
                f"and the arms are holding their observed pose: {startup_blocked}. "
                "Keep this terminal open and press Ctrl+C only when you are ready "
                "to release torque.",
                flush=True,
            )

        period = 1.0 / args.hz
        max_joint_step = args.max_joint_speed / args.hz
        max_gripper_step = args.max_gripper_speed / args.hz
        for name in ARM_NAMES:
            limits = np.asarray(robots[name].get_robot_info()["joint_limits"], dtype=np.float64)

            def validate_target(target: np.ndarray, limits: np.ndarray = limits) -> str | None:
                return joint_limit_reason_from_limits(limits, target, args.joint_limit_margin)

            async_ik[name] = AsyncFallbackIkSolver(
                FallbackIkSolver(
                    solve_models[name].solve,
                    validate_target,
                    fallback_attempts_per_cycle=args.ik_fallback_attempts_per_cycle,
                ),
                name=name,
            )
        joint_goals = {name: joints[name].copy() for name in ARM_NAMES}
        # The state machine only consumes live robot FK when ZERO is captured.  Keep the latest
        # pose that produced a valid IK result for fallback search instead of recomputing FK for
        # both arms on every control tick.  Per-frame FK contends with the background MuJoCo IK
        # workers and can make an otherwise short control cycle miss the dataset deadline.
        ik_reference_poses = {name: fk_models[name].fk(joints[name]) for name in ARM_NAMES}
        ik_generation = 0
        latest_ik_results: dict[str, IkFallbackResult] = {}
        ik_holding_since: dict[str, float | None] = {name: None for name in ARM_NAMES}
        lag_monitor = (
            ControlLagMonitor(
                args.dataset.max_control_lag_s,
                args.dataset.control_lag_grace_s,
            )
            if recorder is not None
            else None
        )
        timing_warning_active = False
        timing_window_started: float | None = None
        timing_samples: dict[str, list[float]] = {
            name: [] for name in ("cycle", "robot_read", "control", "dataset", "dispatch")
        }

        def invalidate_ik() -> None:
            nonlocal ik_generation, joint_goals, latest_ik_results, ik_holding_since
            ik_generation += 1
            joint_goals = {name: joints[name].copy() for name in ARM_NAMES}
            latest_ik_results.clear()
            ik_holding_since = {name: None for name in ARM_NAMES}

        last_log = 0.0
        stop = False

        def _signal(_signum: int, _frame: Any) -> None:
            nonlocal stop
            stop = True

        signal.signal(signal.SIGINT, _signal)
        signal.signal(signal.SIGTERM, _signal)
        next_step = time.monotonic()
        while not stop:
            cycle_started = time.monotonic()
            packet, packet_age = receiver.poll()
            if packet is None or packet_age is None:
                was_armed = state.armed
                state.disarm("waiting for telemetry")
                if was_armed:
                    invalidate_ik()
                if recorder is not None and was_armed:
                    try:
                        recorder.finish_episode(commit=False, reason=state.reason)
                    except DatasetRecordingError as exc:
                        raise SafetyStop(str(exc)) from exc
                command_pair(robots, joints, grips)
                time.sleep(period)
                continue
            current_joints, current_grips = read_pair_state(robots)
            zero_capture_requested = (
                state.session_id == packet["session_id"]
                and packet["seq"] > state.last_seq
                and packet["zero_id"] > state.zero_id
            )
            robot_poses = (
                {name: fk_models[name].fk(current_joints[name]) for name in ARM_NAMES}
                if zero_capture_requested
                else ik_reference_poses
            )
            observation_done = time.monotonic()
            was_armed = state.armed
            previous_session_id = state.session_id
            previous_cancel_id = state.cancel_id
            result = state.step(packet, packet_age, robot_poses)
            if result.armed:
                base_pose_verified = False
            prearm_episode_fault = None
            if recorder is not None and recorder.active and not was_armed and not result.armed:
                if previous_session_id is not None and packet["session_id"] != previous_session_id:
                    prearm_episode_fault = "source restarted before teleoperation enabled"
                elif packet["cancel_id"] > previous_cancel_id:
                    prearm_episode_fault = "cancel button"
                elif result.reason.startswith("telemetry stale"):
                    prearm_episode_fault = result.reason
                elif "stale" in result.reason or result.reason.startswith("ZERO rejected"):
                    prearm_episode_fault = result.reason
                if prearm_episode_fault is not None:
                    try:
                        recorder.finish_episode(commit=False, reason=prearm_episode_fault)
                    except DatasetRecordingError as exc:
                        raise SafetyStop(str(exc)) from exc
            if result.quit:
                if recorder is not None:
                    try:
                        recorder.finish_episode(commit=False, reason=state.reason)
                    except DatasetRecordingError as exc:
                        raise SafetyStop(str(exc)) from exc
                break
            if startup_blocked is not None:
                message = f"startup reset incomplete: {startup_blocked}"
                state.disarm(message)
                if result.zero_captured:
                    reject = f"ZERO rejected: {message}"
                    state.reject_zero(reject)
                    print(reject, flush=True)
                command_pair(robots, joints, grips)
                now = time.monotonic()
                if now - last_log >= 0.5:
                    hold_error = max(
                        float(np.max(np.abs(current_joints[name] - joints[name])))
                        for name in ARM_NAMES
                    )
                    print(
                        f"startup_hold reason={message} packet_age={packet_age:.3f}s "
                        f"hold_err_deg={math.degrees(hold_error):.1f} "
                        f"bad={receiver.bad_packets}",
                        flush=True,
                    )
                    last_log = now
                next_step += period
                time.sleep(max(0.0, next_step - time.monotonic()))
                continue
            if result.zero_captured:
                joints = {name: current_joints[name].copy() for name in ARM_NAMES}
                grips = dict(current_grips)
                ik_reference_poses = {
                    name: np.asarray(robot_poses[name], dtype=np.float64).copy()
                    for name in ARM_NAMES
                }
                invalidate_ik()
                at_verified_base = base_pose_verified and all(
                    float(
                        np.max(
                            np.abs(
                                current_joints[name]
                                - np.asarray(DEFAULT_BASE_JOINTS, dtype=np.float64)
                            )
                        )
                    )
                    <= args.startup_readback_tolerance
                    for name in ARM_NAMES
                )
                for name in ARM_NAMES:
                    reason = (
                        None
                        if at_verified_base
                        else joint_limit_reason(
                            robots[name], current_joints[name], args.joint_limit_margin
                        )
                    )
                    if reason is not None:
                        message = f"ZERO rejected: {name} {reason}; return to the base pose"
                        state.reject_zero(message)
                        print(message, flush=True)
                        break
            held_joints = {name: joints[name].copy() for name in ARM_NAMES}
            held_grips = dict(grips)
            if result.armed and result.targets is not None:
                request_time = time.monotonic()
                for name in ARM_NAMES:
                    async_ik[name].submit(
                        result.targets[name],
                        ik_reference_poses[name],
                        current_joints[name],
                        generation=ik_generation,
                        submitted_at=request_time,
                    )
                cycle_now = time.monotonic()
                for name in ARM_NAMES:
                    completion = async_ik[name].poll(generation=ik_generation)
                    if completion is None:
                        continue
                    if cycle_now - completion.submitted_at > args.ik_result_max_age_s:
                        joint_goals[name] = joints[name].copy()
                        latest_ik_results.pop(name, None)
                        ik_holding_since[name] = None
                        continue
                    outcome = completion.outcome
                    latest_ik_results[name] = outcome
                    joint_goals[name] = (
                        joints[name].copy()
                        if outcome.mode == "holding_ik"
                        else outcome.joints.copy()
                    )
                    if outcome.mode != "holding_ik":
                        ik_reference_poses[name] = outcome.target_pose.copy()
                for name in ARM_NAMES:
                    outcome = latest_ik_results.get(name)
                    if outcome is None:
                        continue
                    if outcome.mode == "holding_ik":
                        if ik_holding_since[name] is None:
                            ik_holding_since[name] = cycle_now
                    else:
                        ik_holding_since[name] = None

                for name in ARM_NAMES:
                    delta = np.clip(
                        joint_goals[name] - joints[name], -max_joint_step, max_joint_step
                    )
                    joints[name] += delta

                    desired_grip = result.grippers[name]
                    if desired_grip is not None:
                        grip_delta = np.clip(
                            desired_grip - grips[name], -max_gripper_step, max_gripper_step
                        )
                        grips[name] = float(np.clip(grips[name] + grip_delta, 0.0, 1.0))

                holding = [
                    name
                    for name, outcome in latest_ik_results.items()
                    if outcome.mode == "holding_ik"
                ]
                degraded = [
                    name
                    for name, outcome in latest_ik_results.items()
                    if outcome.mode == "degraded_ik"
                ]
                if args.ik_hold_disarm_s > 0.0:
                    expired = [
                        name
                        for name in holding
                        if ik_holding_since[name] is not None
                        and cycle_now - ik_holding_since[name] >= args.ik_hold_disarm_s
                    ]
                    if expired:
                        state.disarm(
                            "safety_disarmed: persistent IK fallback failure for "
                            + ",".join(expired)
                            + f" ({args.ik_hold_disarm_s:.1f}s)"
                        )
                if state.armed:
                    if holding:
                        state.reason = (
                            "holding_ik: " + ",".join(holding) + " holding last safe target"
                        )
                    elif degraded:
                        state.reason = "degraded_ik: " + ",".join(degraded) + " using fallback"
                    else:
                        state.reason = result.reason
                    if result.envelope_clamps and (holding or degraded):
                        state.reason += "; " + result.reason
            elif not result.armed:
                joint_goals = {name: joints[name].copy() for name in ARM_NAMES}
                latest_ik_results.clear()
                ik_holding_since = {name: None for name in ARM_NAMES}
            if was_armed and not state.armed:
                invalidate_ik()
            control_done = time.monotonic()
            prepared_dataset_frame = None
            if recorder is not None:
                try:
                    prepared_dataset_frame = prepare_dataset_step(
                        recorder,
                        was_armed=was_armed,
                        is_armed=state.armed,
                        invalid_frame_reason=None,
                        measured_joints=current_joints,
                        measured_grippers=current_grips,
                        commanded_joints=joints,
                        commanded_grippers=grips,
                    )
                except DatasetFrameError as exc:
                    state.disarm(f"dataset episode invalid: {exc}")
                    joints = held_joints
                    grips = held_grips
                    invalidate_ik()
                    try:
                        recorder.finish_episode(commit=False, reason=state.reason)
                    except DatasetRecordingError as finish_exc:
                        raise SafetyStop(str(finish_exc)) from finish_exc
                except DatasetRecordingError as exc:
                    state.disarm(f"dataset failure: {exc}")
                    raise SafetyStop(state.reason) from exc
            dataset_done = time.monotonic()
            command_pair(robots, joints, grips)

            if recorder is not None:
                try:
                    commit_dataset_step(
                        recorder,
                        prepared_dataset_frame,
                        was_armed=was_armed,
                        is_armed=state.armed,
                        disarm_reason=state.reason,
                    )
                except DatasetRecordingError as exc:
                    state.disarm(f"dataset failure: {exc}")
                    raise SafetyStop(state.reason) from exc
            dispatch_done = time.monotonic()

            episode_stopped = (
                recorder is not None
                and was_armed
                and not state.armed
                and state.reason == "enable toggled off"
            )
            if episode_stopped:
                print(
                    "recording ended; returning both YAMs to unrecorded base pose",
                    flush=True,
                )
                invalidate_ik()
                reset_joints, reset_grips = read_pair_state(robots)
                grips = dict(reset_grips)
                try:
                    joints = move_pair_to_base(
                        robots,
                        reset_joints,
                        grips,
                        args.base_joint_speed,
                        args.hz,
                        args.startup_readback_tolerance,
                    )
                    base_pose_verified = True
                    startup_blocked = None
                    state.invalidate_zero("returned to base pose; tap ZERO")
                    ik_reference_poses = {
                        name: fk_models[name].fk(joints[name]) for name in ARM_NAMES
                    }
                    joint_goals = {name: joints[name].copy() for name in ARM_NAMES}
                    command_pair(robots, joints, grips)
                    print(
                        "robots at unrecorded base pose; tap ZERO, then enable for a new episode",
                        flush=True,
                    )
                except StartupReadbackError as exc:
                    joints = {name: exc.observed[name].copy() for name in ARM_NAMES}
                    base_pose_verified = False
                    startup_blocked = f"post-episode base reset failed: {exc}"
                    state.invalidate_zero(startup_blocked)
                    command_pair(robots, joints, grips)
                    print(
                        "RESET HOLD: automatic base return did not verify; arms are holding "
                        f"their observed pose and enable is locked out: {exc}",
                        flush=True,
                    )
                if lag_monitor is not None:
                    lag_monitor.reset()
                timing_warning_active = False
                timing_window_started = None
                for samples in timing_samples.values():
                    samples.clear()
                next_step = time.monotonic()
                last_log = 0.0
                continue

            if state.armed or was_armed:
                if timing_window_started is None:
                    timing_window_started = cycle_started
                for name, duration in (
                    ("cycle", dispatch_done - cycle_started),
                    ("robot_read", observation_done - cycle_started),
                    ("control", control_done - observation_done),
                    ("dataset", dataset_done - control_done),
                    ("dispatch", dispatch_done - dataset_done),
                ):
                    timing_samples[name].append(duration * 1000.0)
                timing_duration = dispatch_done - timing_window_started
                if timing_duration >= 0.5:
                    pieces = []
                    for name, samples in timing_samples.items():
                        percentiles = np.percentile(samples, (50, 95, 99))
                        pieces.append(
                            f"{name}[p50={percentiles[0]:.1f} p95={percentiles[1]:.1f} "
                            f"p99={percentiles[2]:.1f}]ms"
                        )
                    queue_text = ""
                    if recorder is not None:
                        queue_text = (
                            " queues["
                            + " ".join(
                                f"{name}={depth}" for name, depth in recorder.queue_depths().items()
                            )
                            + "]"
                        )
                    print(
                        f"TIMING effective_hz={len(timing_samples['cycle']) / timing_duration:.1f} "
                        + " ".join(pieces)
                        + queue_text,
                        flush=True,
                    )
                    timing_window_started = None
                    for samples in timing_samples.values():
                        samples.clear()
            else:
                timing_window_started = None
                for samples in timing_samples.values():
                    samples.clear()

            now = time.monotonic()
            if now - last_log >= 0.5:
                input_text = ""
                if result.input_deltas is not None:
                    pieces = []
                    for name in ARM_NAMES:
                        delta = result.input_deltas[name]
                        dxyz_mm = 1000.0 * delta[:3, 3]
                        pieces.append(
                            f"{name[0].upper()}=[{dxyz_mm[0]:+.1f},"
                            f"{dxyz_mm[1]:+.1f},{dxyz_mm[2]:+.1f}]mm/"
                            f"{math.degrees(rotation_angle(delta[:3, :3])):.1f}deg"
                        )
                    input_text = " input[" + " ".join(pieces) + "]"
                clamp_text = ""
                if result.envelope_clamps is not None and result.command_deltas is not None:
                    pieces = []
                    for name, reasons in result.envelope_clamps.items():
                        delta = result.command_deltas[name]
                        dxyz_mm = 1000.0 * delta[:3, 3]
                        pieces.append(
                            f"{name[0].upper()}=[{dxyz_mm[0]:+.1f},"
                            f"{dxyz_mm[1]:+.1f},{dxyz_mm[2]:+.1f}]mm/"
                            f"{math.degrees(rotation_angle(delta[:3, :3])):.1f}deg "
                            f"({' '.join(reasons)})"
                        )
                    clamp_text = " command_clamped[" + " | ".join(pieces) + "]"
                ik_text = ""
                if latest_ik_results and result.targets is not None:
                    ik_text = (
                        " ik["
                        + " | ".join(
                            fallback_diagnostic(name, latest_ik_results[name], result.targets[name])
                            for name in ARM_NAMES
                            if name in latest_ik_results
                        )
                        + "]"
                    )
                print(
                    f"armed={state.armed} zeroed={state.zeroed} reason={state.reason} "
                    f"grip[L={grips['left']:.3f} R={grips['right']:.3f}] "
                    f"packet_age={packet_age:.3f}s bad={receiver.bad_packets}"
                    f"{input_text}{clamp_text}{ik_text}",
                    flush=True,
                )
                last_log = now
            next_step += period
            timing_now = time.monotonic()
            control_lag = max(0.0, timing_now - next_step)
            cycle_overrun = max(0.0, dispatch_done - cycle_started - period)
            if recorder is not None and recorder.active:
                assert lag_monitor is not None
                decision = lag_monitor.observe(
                    control_lag,
                    cycle_overrun_s=cycle_overrun,
                    now=timing_now,
                )
                if decision.over_limit and not timing_warning_active:
                    print(
                        f"DATASET timing recovery lag={control_lag:.3f}s; "
                        f"allowing {args.dataset.control_lag_grace_s:.3f}s to catch up",
                        flush=True,
                    )
                    timing_warning_active = True
                elif not decision.over_limit:
                    if timing_warning_active:
                        print("DATASET timing recovered", flush=True)
                    timing_warning_active = False
                if not decision.fault:
                    time.sleep(max(0.0, next_step - time.monotonic()))
                    continue
                reason = (
                    f"control loop lagged by {control_lag:.3f}s for "
                    f"{decision.over_limit_for_s:.3f}s; "
                    f"last cycle={(dispatch_done - cycle_started) * 1000:.1f}ms "
                    f"robot_read={(observation_done - cycle_started) * 1000:.1f}ms "
                    f"control/IK={(control_done - observation_done) * 1000:.1f}ms "
                    f"dataset={(dataset_done - control_done) * 1000:.1f}ms "
                    f"dispatch={(dispatch_done - dataset_done) * 1000:.1f}ms"
                )
                state.disarm(f"dataset episode invalid: {reason}")
                invalidate_ik()
                try:
                    recorder.finish_episode(
                        commit=False,
                        reason=state.reason,
                    )
                except DatasetRecordingError as exc:
                    raise SafetyStop(str(exc)) from exc
                timing_warning_active = False
                lag_monitor.reset()
            else:
                if lag_monitor is not None:
                    lag_monitor.reset()
                timing_warning_active = False
            time.sleep(max(0.0, next_step - time.monotonic()))
    finally:
        pending_exception = sys.exc_info()[0] is not None
        cleanup_errors: list[str] = []
        for name in reversed(ARM_NAMES):
            if name in async_ik:
                async_ik[name].close()
        for name in reversed(ARM_NAMES):
            if name in robots:
                try:
                    close_robot(robots[name])
                except Exception as exc:
                    cleanup_errors.append(f"failed to close {name} robot: {exc}")
        for name in reversed(ARM_NAMES):
            if name in async_ik and not async_ik[name].close(timeout_s=1.0):
                cleanup_errors.append(f"{name} IK worker did not stop within 1.0s")
        try:
            receiver.close()
        except Exception as exc:
            cleanup_errors.append(f"failed to close telemetry receiver: {exc}")
        for lock in locks:
            try:
                lock.close()
            except Exception as exc:
                cleanup_errors.append(f"failed to close CAN lock: {exc}")
        if recorder is not None:
            try:
                recorder.close(upload=not pending_exception and not cleanup_errors)
            except DatasetRecordingError as exc:
                cleanup_errors.append(f"dataset cleanup failed: {exc}")
        if cleanup_errors:
            detail = "; ".join(cleanup_errors)
            if pending_exception:
                print(f"CLEANUP ERROR: {detail}", file=sys.stderr, flush=True)
            else:
                raise SafetyStop(detail)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    try:
        run(tyro.cli(Config))
    except SafetyStop as exc:
        print(f"SAFETY STOP: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()

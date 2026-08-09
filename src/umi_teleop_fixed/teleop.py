"""Safe, relative, dual-arm teleoperation from two libsurvive trackers."""

from __future__ import annotations

import math
import os
import select
import sys
import termios
import time
import tty
from collections.abc import Callable, Mapping
from contextlib import ExitStack
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Literal, Protocol

import numpy as np
import tyro
from numpy.typing import NDArray

from umi_teleop_fixed.calibration import (
    UmiGripperCalibration,
    cache_root,
    load_umi_gripper_calibrations,
    umi_gripper_calibration_path,
)
from umi_teleop_fixed.debug import (
    DebugRecorder,
    JsonlDebugRecorder,
    NullDebugRecorder,
    timestamped_debug_path,
)
from umi_teleop_fixed.gripper_receiver import GripperReceiver, ReceivedGripperSample
from umi_teleop_fixed.lighthouses import (
    load_lighthouse_calibration_id,
    require_lighthouse_calibration,
)
from umi_teleop_fixed.robot import (
    DEFAULT_START_POSITION,
    SIDES,
    YAM_JOINT_LIMITS,
    ZERO_POSITION,
    ArmState,
    DualYamBackend,
    validate_dual_position,
)
from umi_teleop_fixed.se3 import (
    PoseMatrix,
    pose_matrix,
    rate_limit_pose,
    rotation_distance,
    rpy_degrees_to_matrix,
    scaled_relative_target,
    workspace_error,
)
from umi_teleop_fixed.teleop_frame import load_teleop_frame, teleop_frame_path
from umi_teleop_fixed.trackers import (
    LibsurviveSource,
    PoseSource,
    TrackerReader,
    TrackerSample,
)


@dataclass(frozen=True)
class Args:
    """Teleoperate two YAM arms from two Vive trackers."""

    robot_id: str
    """Robot identity used to load cached gripper calibration."""

    left_adapter_serial: str
    """Stable USB serial for the CAN adapter connected to the left arm."""

    right_adapter_serial: str
    """Stable USB serial for the CAN adapter connected to the right arm."""

    left_tracker_serial: str
    """Libsurvive serial of the physical left tracker."""

    right_tracker_serial: str
    """Libsurvive serial of the physical right tracker."""

    start_position: tuple[float, ...] = DEFAULT_START_POSITION
    """Left joints/gripper followed by right joints/gripper; 14 values total."""

    mode: Literal["dry-run", "active"] = "dry-run"
    """Dry-run computes full IK but sends no position commands."""

    control_rate_hz: float = 60.0
    """Cartesian control-loop rate."""

    translation_scale: float = 1.0
    """Multiplier for translation relative to the engage pose."""

    rotation_scale: float = 1.0
    """Multiplier for rotation relative to engage; use 0 for translation-only control."""

    tracker_to_robot_rpy_degrees: tuple[float, float, float] | None = None
    """Explicit room-frame override; defaults to the robot's cached calibration."""

    max_linear_speed_m_s: float = 0.2
    """Maximum Cartesian target speed per arm."""

    max_angular_speed_rad_s: float = 1.0
    """Maximum angular target speed per arm."""

    max_translation_from_start_m: float = 0.3
    """Maximum target translation from the pose captured at engage."""

    max_rotation_from_start_rad: float = 1.57
    """Maximum target rotation from the pose captured at engage."""

    tracker_timeout_seconds: float = 0.1
    """Pose age that immediately disengages both arms."""

    gripper_sender_ip: str | None = None
    """Expected Pi IPv4 address; omit to hold the robot grippers at engage position."""

    gripper_port: int = 9130
    """UDP port carrying versioned raw-width packets."""

    gripper_bind_host: str = "0.0.0.0"
    """Local address on which to receive gripper packets."""

    max_gripper_speed_per_s: float = 1.5
    """Maximum normalized gripper-command change per second."""

    gripper_timeout_seconds: float = 0.5
    """Maximum combined camera and network age before the gripper freezes."""

    libsurvive_path: Path | None = None
    """Path to libsurvive.so when it is not installed in a standard path."""

    config_path: Path | None = None
    """Libsurvive config path; defaults to the user cache."""

    poser: str = "MPFIT"
    """Libsurvive poser plugin."""

    debug: bool = False
    """Write per-cycle structured telemetry for post-run debugging."""

    debug_log_path: Path | None = None
    """Debug JSONL destination; defaults to the user cache when --debug is set."""

    def validate(self) -> None:
        identifiers = {
            "robot_id": self.robot_id,
            "left_adapter_serial": self.left_adapter_serial,
            "right_adapter_serial": self.right_adapter_serial,
            "left_tracker_serial": self.left_tracker_serial,
            "right_tracker_serial": self.right_tracker_serial,
            "poser": self.poser,
        }
        for name, value in identifiers.items():
            if not value.strip():
                raise ValueError(f"{name} must not be empty")
        if self.left_adapter_serial.casefold() == self.right_adapter_serial.casefold():
            raise ValueError("left and right CAN adapter serials must be different")
        if self.left_tracker_serial.casefold() == self.right_tracker_serial.casefold():
            raise ValueError("left and right tracker serials must be different")
        positive = {
            "control_rate_hz": self.control_rate_hz,
            "translation_scale": self.translation_scale,
            "max_linear_speed_m_s": self.max_linear_speed_m_s,
            "max_angular_speed_rad_s": self.max_angular_speed_rad_s,
            "max_translation_from_start_m": self.max_translation_from_start_m,
            "max_rotation_from_start_rad": self.max_rotation_from_start_rad,
            "tracker_timeout_seconds": self.tracker_timeout_seconds,
            "max_gripper_speed_per_s": self.max_gripper_speed_per_s,
            "gripper_timeout_seconds": self.gripper_timeout_seconds,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        if not math.isfinite(self.rotation_scale) or self.rotation_scale < 0:
            raise ValueError("rotation_scale must be nonnegative")
        if self.tracker_to_robot_rpy_degrees is not None:
            rpy_degrees_to_matrix(self.tracker_to_robot_rpy_degrees)
        validate_dual_position(self.start_position)
        if self.debug_log_path is not None and not self.debug:
            raise ValueError("debug_log_path requires --debug")
        if not 1 <= self.gripper_port <= 65535:
            raise ValueError("gripper_port must be between 1 and 65535")

    @property
    def gripper_enabled(self) -> bool:
        return self.gripper_sender_ip is not None

    def resolved_config_path(self) -> Path:
        if self.config_path is not None:
            return self.config_path.expanduser()
        return cache_root() / "umi-teleop-fixed" / "libsurvive" / "config.json"

    def resolved_debug_log_path(self) -> Path:
        if self.debug_log_path is not None:
            return self.debug_log_path.expanduser()
        return timestamped_debug_path(cache_root() / "umi-teleop-fixed")

    def resolved_umi_gripper_calibration_path(self) -> Path:
        return umi_gripper_calibration_path(self.robot_id)

    def resolved_tracker_to_robot_rpy_degrees(self) -> tuple[float, float, float]:
        if self.tracker_to_robot_rpy_degrees is not None:
            return self.tracker_to_robot_rpy_degrees
        require_lighthouse_calibration(self.resolved_config_path())
        calibration = load_teleop_frame(
            teleop_frame_path(self.robot_id),
            expected_robot_id=self.robot_id,
            expected_lighthouse_calibration_id=load_lighthouse_calibration_id(
                self.resolved_config_path()
            ),
        )
        return calibration.rpy_degrees


@dataclass
class _EngagedSide:
    tracker_pose: PoseMatrix
    robot_pose: PoseMatrix
    last_requested: PoseMatrix
    last_target: PoseMatrix
    last_joints: NDArray[np.float64]
    joint_correction: NDArray[np.float64]
    gripper_position: float
    umi_width_mm: float | None
    umi_stream_id: str | None
    last_gripper_target: float
    gripper_latched: bool


JOINT_TRACKING_INTEGRAL_GAIN_PER_SECOND = 1.5
MAX_JOINT_TRACKING_CORRECTION_RAD = 0.05


class TeleopController:
    """Pure engage-relative target generation with explicit commit semantics."""

    def __init__(
        self,
        args: Args,
        *,
        gripper_calibrations: Mapping[str, UmiGripperCalibration] | None = None,
    ) -> None:
        self._args = args
        if args.tracker_to_robot_rpy_degrees is None:
            raise RuntimeError("teleop frame must be resolved before constructing the controller")
        self._input_to_robot_rotation = rpy_degrees_to_matrix(
            args.tracker_to_robot_rpy_degrees
        )
        self._engaged: dict[str, _EngagedSide] | None = None
        self._gripper_calibrations = gripper_calibrations
        if args.gripper_enabled and (
            gripper_calibrations is None or set(gripper_calibrations) != set(SIDES)
        ):
            raise RuntimeError("left and right UMI gripper calibrations are required")

    @property
    def engaged(self) -> bool:
        return self._engaged is not None

    def engage(
        self,
        states: Mapping[str, ArmState],
        samples: Mapping[str, TrackerSample],
        gripper_samples: Mapping[str, ReceivedGripperSample] | None = None,
    ) -> None:
        if self._args.gripper_enabled and gripper_samples is None:
            raise RuntimeError("fresh gripper widths are required to engage")
        self._engaged = {}
        for side in SIDES:
            tracker = samples[side].pose
            gripper_sample = None if gripper_samples is None else gripper_samples[side]
            self._engaged[side] = _EngagedSide(
                tracker_pose=pose_matrix(tracker.position, tracker.quaternion_wxyz),
                robot_pose=states[side].ee_pose.copy(),
                last_requested=states[side].ee_pose.copy(),
                last_target=states[side].ee_pose.copy(),
                last_joints=states[side].joint_positions.copy(),
                joint_correction=np.zeros(6, dtype=np.float64),
                gripper_position=states[side].gripper_position,
                umi_width_mm=None if gripper_sample is None else gripper_sample.width_mm,
                umi_stream_id=None if gripper_sample is None else gripper_sample.stream_id,
                last_gripper_target=states[side].gripper_position,
                gripper_latched=False,
            )

    def disengage(self) -> None:
        self._engaged = None

    def propose_detailed(
        self,
        samples: Mapping[str, TrackerSample],
        *,
        dt: float,
    ) -> TargetProposal:
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        requested_targets = {}
        for side in SIDES:
            baseline = self._engaged[side]
            tracker = samples[side].pose
            requested_targets[side] = scaled_relative_target(
                baseline.tracker_pose,
                pose_matrix(tracker.position, tracker.quaternion_wxyz),
                baseline.robot_pose,
                translation_scale=self._args.translation_scale,
                rotation_scale=self._args.rotation_scale,
                input_to_robot_rotation=self._input_to_robot_rotation,
            )
        command_targets = {}
        for side in SIDES:
            baseline = self._engaged[side]
            requested = requested_targets[side]
            error = workspace_error(
                baseline.robot_pose,
                requested,
                max_translation_m=self._args.max_translation_from_start_m,
                max_rotation_rad=self._args.max_rotation_from_start_rad,
            )
            if error is not None:
                raise WorkspaceViolation(side, error, requested_targets)
            command_targets[side] = rate_limit_pose(
                baseline.last_target,
                requested,
                dt=dt,
                max_linear_speed_m_s=self._args.max_linear_speed_m_s,
                max_angular_speed_rad_s=self._args.max_angular_speed_rad_s,
            )
        return TargetProposal(requested=requested_targets, command=command_targets)

    def propose(
        self,
        samples: Mapping[str, TrackerSample],
        *,
        dt: float,
    ) -> dict[str, PoseMatrix]:
        """Return only rate-limited targets for simple callers and tests."""
        return self.propose_detailed(samples, dt=dt).command

    def seeds(self) -> dict[str, NDArray[np.float64]]:
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        return {side: self._engaged[side].last_joints.copy() for side in SIDES}

    def grippers(self) -> dict[str, float]:
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        return {side: self._engaged[side].last_gripper_target for side in SIDES}

    def propose_grippers(
        self,
        samples: Mapping[str, ReceivedGripperSample] | None,
        *,
        dt: float,
        unavailable_reasons: Mapping[str, str] | None = None,
    ) -> GripperTargetProposal:
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive")
        requested: dict[str, float] = {}
        command: dict[str, float] = {}
        status: dict[str, str] = {}
        for side in SIDES:
            engaged = self._engaged[side]
            requested[side] = engaged.last_gripper_target
            command[side] = engaged.last_gripper_target
            if not self._args.gripper_enabled:
                status[side] = "held (stream disabled)"
                continue
            if engaged.gripper_latched:
                status[side] = "held (re-zero required)"
                continue
            if samples is None or side not in samples:
                reason = None if unavailable_reasons is None else unavailable_reasons.get(side)
                status[side] = f"held ({reason or 'width unavailable'})"
                continue
            sample = samples[side]
            if sample.stream_id != engaged.umi_stream_id:
                engaged.gripper_latched = True
                status[side] = "held (Pi stream restarted; re-zero required)"
                continue
            assert engaged.umi_width_mm is not None
            assert self._gripper_calibrations is not None
            span_mm = self._gripper_calibrations[side].span_mm
            raw_target = engaged.gripper_position + (
                sample.width_mm - engaged.umi_width_mm
            ) / span_mm
            requested[side] = float(np.clip(raw_target, 0.0, 1.0))
            max_step = self._args.max_gripper_speed_per_s * dt
            command[side] = float(
                np.clip(
                    requested[side],
                    engaged.last_gripper_target - max_step,
                    engaged.last_gripper_target + max_step,
                )
            )
            status[side] = "tracking"
        return GripperTargetProposal(requested=requested, command=command, status=status)

    def validate_measured_workspace(self, states: Mapping[str, ArmState]) -> None:
        """Stop if feedback shows that a physical TCP has left the engage workspace."""
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        for side in SIDES:
            error = workspace_error(
                self._engaged[side].robot_pose,
                states[side].ee_pose,
                max_translation_m=self._args.max_translation_from_start_m,
                max_rotation_rad=self._args.max_rotation_from_start_rad,
            )
            if error is not None:
                raise RuntimeError(f"{side} measured physical TCP left the safe workspace: {error}")

    def compensated_joint_commands(
        self,
        desired: Mapping[str, NDArray[np.float64]],
        states: Mapping[str, ArmState],
        *,
        dt: float,
    ) -> dict[str, NDArray[np.float64]]:
        """Close the steady-state joint-error loop around i2rt's position controller."""
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive")
        limits = np.asarray(YAM_JOINT_LIMITS, dtype=np.float64)
        commands = {}
        for side in SIDES:
            target = np.asarray(desired[side], dtype=np.float64)
            measured = np.asarray(states[side].joint_positions, dtype=np.float64)
            if target.shape != (6,) or measured.shape != (6,):
                raise RuntimeError("joint compensation requires six arm joints per side")
            engaged = self._engaged[side]
            engaged.joint_correction += (
                JOINT_TRACKING_INTEGRAL_GAIN_PER_SECOND * dt * (target - measured)
            )
            engaged.joint_correction = np.clip(
                engaged.joint_correction,
                -MAX_JOINT_TRACKING_CORRECTION_RAD,
                MAX_JOINT_TRACKING_CORRECTION_RAD,
            )
            commands[side] = np.clip(
                target + engaged.joint_correction,
                limits[:, 0],
                limits[:, 1],
            )
        return commands

    def joint_corrections(self) -> dict[str, NDArray[np.float64]]:
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        return {side: self._engaged[side].joint_correction.copy() for side in SIDES}

    def commit(
        self,
        targets: Mapping[str, PoseMatrix],
        joint_targets: Mapping[str, NDArray[np.float64]],
        *,
        requested_targets: Mapping[str, PoseMatrix] | None = None,
        gripper_targets: Mapping[str, float] | None = None,
    ) -> None:
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        for side in SIDES:
            requested = targets[side] if requested_targets is None else requested_targets[side]
            self._engaged[side].last_requested = requested.copy()
            self._engaged[side].last_target = targets[side].copy()
            self._engaged[side].last_joints = joint_targets[side].copy()
            if gripper_targets is not None:
                self._engaged[side].last_gripper_target = float(gripper_targets[side])

    def motion_summary(self) -> str:
        """Describe the latest engage-relative Cartesian target on both sides."""
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        summaries = []
        for side in SIDES:
            state = self._engaged[side]
            requested_mm = (
                state.last_requested[:3, 3] - state.robot_pose[:3, 3]
            ) * 1000.0
            command_mm = (state.last_target[:3, 3] - state.robot_pose[:3, 3]) * 1000.0
            requested_rotation = math.degrees(
                rotation_distance(state.robot_pose, state.last_requested)
            )
            command_rotation = math.degrees(
                rotation_distance(state.robot_pose, state.last_target)
            )
            requested_vector = ",".join(f"{value:+.1f}" for value in requested_mm)
            command_vector = ",".join(f"{value:+.1f}" for value in command_mm)
            summaries.append(
                f"{side} request_mm=[{requested_vector}] request_rot={requested_rotation:.1f}deg "
                f"command_mm=[{command_vector}] command_rot={command_rotation:.1f}deg"
            )
        return "  ".join(summaries)


@dataclass(frozen=True)
class TargetProposal:
    """Raw engage-relative requests and their rate-limited command targets."""

    requested: dict[str, PoseMatrix]
    command: dict[str, PoseMatrix]


@dataclass(frozen=True)
class GripperTargetProposal:
    """Raw engage-relative gripper requests and rate-limited command targets."""

    requested: dict[str, float]
    command: dict[str, float]
    status: dict[str, str]


class WorkspaceViolation(RuntimeError):
    """A rejected request carrying the targets needed for post-run diagnosis."""

    def __init__(
        self,
        side: str,
        detail: str,
        requested_targets: Mapping[str, PoseMatrix],
    ) -> None:
        super().__init__(f"{side} tracker target left the safe workspace: {detail}")
        self.requested_targets = {
            name: target.copy() for name, target in requested_targets.items()
        }


class Backend(Protocol):
    def read_states(self) -> dict[str, ArmState]: ...

    def solve_ik(
        self,
        targets: Mapping[str, PoseMatrix],
        seeds: Mapping[str, NDArray[np.float64]],
    ) -> dict[str, NDArray[np.float64]]: ...

    def command(
        self,
        joint_targets: Mapping[str, NDArray[np.float64]],
        gripper_targets: Mapping[str, float],
    ) -> None: ...

    def idle(self) -> None: ...


class TrackerSnapshots(Protocol):
    def snapshot(self, *, now: float, stale_after_seconds: float) -> dict[str, TrackerSample]: ...


class GripperSnapshots(Protocol):
    def snapshot(
        self, *, now: float, stale_after_seconds: float
    ) -> dict[str, ReceivedGripperSample]: ...

    def read(
        self, *, now: float, stale_after_seconds: float
    ) -> tuple[dict[str, ReceivedGripperSample], dict[str, str]]: ...


class KeySource(Protocol):
    def poll(self) -> tuple[str, ...]: ...


class KeyboardInput:
    """Single-key terminal input that restores terminal settings on every exit."""

    def __init__(self) -> None:
        self._descriptor = sys.stdin.fileno()
        self._original: list | None = None

    def __enter__(self) -> KeyboardInput:
        if not os.isatty(self._descriptor):
            raise RuntimeError("umi-teleop requires an interactive terminal")
        self._original = termios.tcgetattr(self._descriptor)
        tty.setcbreak(self._descriptor)
        return self

    def poll(self) -> tuple[str, ...]:
        keys = []
        while select.select([self._descriptor], [], [], 0.0)[0]:
            keys.append(os.read(self._descriptor, 1).decode(errors="ignore").casefold())
        return tuple(keys)

    def __exit__(self, *_exc_info: object) -> None:
        if self._original is not None:
            termios.tcsetattr(self._descriptor, termios.TCSADRAIN, self._original)


def control_loop(
    args: Args,
    *,
    backend: Backend,
    trackers: TrackerSnapshots,
    grippers: GripperSnapshots | None = None,
    gripper_calibrations: Mapping[str, UmiGripperCalibration] | None = None,
    keys: KeySource,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    emit: Callable[[str], None] = print,
    max_cycles: int | None = None,
    initial_idle: bool = True,
    debug_recorder: DebugRecorder | None = None,
) -> None:
    """Run an already-connected system; dependency injection keeps E2E tests deterministic."""
    controller = TeleopController(args, gripper_calibrations=gripper_calibrations)
    debug_log = debug_recorder or NullDebugRecorder()
    period = 1.0 / args.control_rate_hz
    next_tick = monotonic()
    status_started = next_tick
    successful_updates = 0
    gripper_status_summary = ""
    cycles = 0
    if args.gripper_enabled and grippers is None:
        raise RuntimeError("gripper control is enabled but no receiver is running")
    if initial_idle:
        backend.idle()
    emit(f"Ready in {args.mode} mode. Keys: e engage, d disengage, r re-zero, q quit.")
    debug_log.event("control_loop_start", mode=args.mode, initial_idle=initial_idle)
    try:
        while max_cycles is None or cycles < max_cycles:
            cycles += 1
            quit_requested = False
            for key in keys.poll():
                debug_log.event("key", cycle=cycles, key=key)
                if key == "q" or key == "\x03":
                    quit_requested = True
                elif key == "d" and controller.engaged:
                    controller.disengage()
                    backend.idle()
                    debug_log.event("disengage", cycle=cycles, reason="operator")
                    emit("DISENGAGED: both arms are in gravity compensation.")
                elif key in {"e", "r"}:
                    samples = trackers.snapshot(
                        now=monotonic(),
                        stale_after_seconds=args.tracker_timeout_seconds,
                    )
                    states = backend.read_states()
                    gripper_samples = None
                    if grippers is not None:
                        try:
                            gripper_samples = grippers.snapshot(
                                now=monotonic(),
                                stale_after_seconds=args.gripper_timeout_seconds,
                            )
                        except RuntimeError as error:
                            debug_log.event(
                                "engage_refused",
                                cycle=cycles,
                                action="re-zero" if key == "r" else "engage",
                                error=error,
                            )
                            emit(f"ENGAGE REFUSED: {error}.")
                            continue
                    controller.engage(states, samples, gripper_samples)
                    debug_log.event(
                        "engage",
                        cycle=cycles,
                        action="re-zero" if key == "r" else "engage",
                        tracker_samples=samples,
                        robot_states=states,
                        gripper_samples=gripper_samples,
                    )
                    status_started = monotonic()
                    successful_updates = 0
                    action = "RE-ZEROED" if key == "r" else "ENGAGED"
                    emit(f"{action}: captured current tracker and robot poses ({args.mode}).")
            if quit_requested:
                debug_log.event("quit", cycle=cycles)
                return

            if controller.engaged:
                samples = None
                states = None
                try:
                    samples = trackers.snapshot(
                        now=monotonic(),
                        stale_after_seconds=args.tracker_timeout_seconds,
                    )
                    states = backend.read_states()
                    gripper_samples = None
                    gripper_errors = None
                    if grippers is not None:
                        gripper_samples, gripper_errors = grippers.read(
                            now=monotonic(),
                            stale_after_seconds=args.gripper_timeout_seconds,
                        )
                    seeds = controller.seeds()
                    proposal = controller.propose_detailed(samples, dt=period)
                    controller.validate_measured_workspace(states)
                    joints = backend.solve_ik(proposal.command, seeds)
                    gripper_proposal = controller.propose_grippers(
                        gripper_samples,
                        dt=period,
                        unavailable_reasons=gripper_errors,
                    )
                    gripper_targets = gripper_proposal.command
                    gripper_status_summary = "  ".join(
                        f"{side}_grip={gripper_targets[side]:.3f} "
                        f"{gripper_proposal.status[side]}"
                        for side in SIDES
                    )
                    joint_commands = (
                        controller.compensated_joint_commands(joints, states, dt=period)
                        if args.mode == "active"
                        else {side: joints[side].copy() for side in SIDES}
                    )
                    debug_log.event(
                        "control_update",
                        cycle=cycles,
                        tracker_samples=samples,
                        robot_states=states,
                        requested_targets=proposal.requested,
                        command_targets=proposal.command,
                        ik_seeds=seeds,
                        ik_solutions=joints,
                        joint_commands=joint_commands,
                        joint_corrections=controller.joint_corrections(),
                        gripper_samples=gripper_samples,
                        gripper_errors=gripper_errors,
                        gripper_requested_targets=gripper_proposal.requested,
                        gripper_targets=gripper_targets,
                        gripper_status=gripper_proposal.status,
                        command_requested=args.mode == "active",
                    )
                    if args.mode == "active":
                        backend.command(joint_commands, gripper_targets)
                        debug_log.event("command_sent", cycle=cycles)
                    controller.commit(
                        proposal.command,
                        joints,
                        requested_targets=proposal.requested,
                        gripper_targets=gripper_targets,
                    )
                    successful_updates += 1
                except (RuntimeError, ValueError) as error:
                    debug_log.event(
                        "safety_stop",
                        cycle=cycles,
                        error=error,
                        tracker_samples=samples,
                        robot_states=states,
                        requested_targets=getattr(error, "requested_targets", None),
                    )
                    controller.disengage()
                    backend.idle()
                    emit(f"SAFETY STOP: {error}. Both arms returned to gravity compensation.")

            now = monotonic()
            status_elapsed = now - status_started
            if controller.engaged and status_elapsed >= 1.0:
                rate = successful_updates / status_elapsed
                emit(
                    f"RUNNING {args.mode}: dual FK/IK healthy at {rate:.1f} Hz; "
                    f"{controller.motion_summary()}  {gripper_status_summary}"
                )
                status_started = now
                successful_updates = 0

            next_tick += period
            delay = next_tick - now
            if delay > 0:
                sleep(delay)
            else:
                next_tick = monotonic()
    finally:
        controller.disengage()
        backend.idle()
        debug_log.event("control_loop_end", cycles=cycles)


BackendFactory = Callable[[Args], DualYamBackend]
SourceFactory = Callable[[Args], PoseSource]
KeyboardFactory = Callable[[], KeyboardInput]
GripperFactory = Callable[[Args], GripperReceiver]


def _create_backend(args: Args) -> DualYamBackend:
    return DualYamBackend.open(
        robot_id=args.robot_id,
        left_adapter_serial=args.left_adapter_serial,
        right_adapter_serial=args.right_adapter_serial,
    )


def _create_source(args: Args) -> PoseSource:
    config_path = args.resolved_config_path()
    require_lighthouse_calibration(config_path)
    return LibsurviveSource(
        library_path=args.libsurvive_path,
        config_path=config_path,
        poser=args.poser,
        disable_calibration=True,
    )


def _create_gripper_receiver(args: Args) -> GripperReceiver:
    if args.gripper_sender_ip is None:
        raise RuntimeError("gripper receiver requested without --gripper-sender-ip")
    return GripperReceiver(
        sender_ip=args.gripper_sender_ip,
        port=args.gripper_port,
        bind_host=args.gripper_bind_host,
    )


def _run_session(
    args: Args,
    *,
    backend_factory: BackendFactory = _create_backend,
    source_factory: SourceFactory = _create_source,
    keyboard_factory: KeyboardFactory = KeyboardInput,
    gripper_factory: GripperFactory = _create_gripper_receiver,
    gripper_calibrations: Mapping[str, UmiGripperCalibration] | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    emit: Callable[[str], None] = print,
    debug_recorder: DebugRecorder,
) -> None:
    """Run one already-validated session with an open debug recorder."""
    source = source_factory(args)
    with ExitStack() as stack:
        gripper_reader = (
            stack.enter_context(gripper_factory(args)) if args.gripper_enabled else None
        )
        reader = stack.enter_context(
            TrackerReader(
                source,
                serials={"left": args.left_tracker_serial, "right": args.right_tracker_serial},
                monotonic=monotonic,
            )
        )
        emit("Waiting for fresh poses from both trackers...")
        deadline = monotonic() + 5.0
        last_error = "no poses received"
        while monotonic() < deadline:
            try:
                reader.snapshot(
                    now=monotonic(),
                    stale_after_seconds=args.tracker_timeout_seconds,
                )
                break
            except RuntimeError as error:
                last_error = str(error)
                sleep(0.01)
        else:
            raise RuntimeError(f"trackers were not ready after 5 seconds: {last_error}")

        with backend_factory(args) as backend:
            if args.mode == "dry-run":
                emit(
                    "Dry-run validated start_position; startup and zero-return trajectories "
                    "will not send commands."
                )
                with keyboard_factory() as keyboard:
                    control_loop(
                        args,
                        backend=backend,
                        trackers=reader,
                        grippers=gripper_reader,
                        gripper_calibrations=gripper_calibrations,
                        keys=keyboard,
                        monotonic=monotonic,
                        sleep=sleep,
                        emit=emit,
                        debug_recorder=debug_recorder,
                    )
            else:
                debug_recorder.event("move_to_start_begin", position=args.start_position)
                backend.move_to_position(
                    args.start_position,
                    monotonic=monotonic,
                    sleep=sleep,
                    emit=emit,
                )
                debug_recorder.event("move_to_start_complete", position=args.start_position)
                try:
                    with keyboard_factory() as keyboard:
                        control_loop(
                            args,
                            backend=backend,
                            trackers=reader,
                            grippers=gripper_reader,
                            gripper_calibrations=gripper_calibrations,
                            keys=keyboard,
                            monotonic=monotonic,
                            sleep=sleep,
                            emit=emit,
                            initial_idle=False,
                            debug_recorder=debug_recorder,
                        )
                finally:
                    emit("Returning both arms to zero_position before shutdown...")
                    debug_recorder.event("return_to_zero_begin", position=ZERO_POSITION)
                    backend.move_to_position(
                        ZERO_POSITION,
                        position_name="zero_position",
                        monotonic=monotonic,
                        sleep=sleep,
                        emit=emit,
                    )
                    debug_recorder.event("return_to_zero_complete", position=ZERO_POSITION)


def run(
    args: Args,
    *,
    backend_factory: BackendFactory = _create_backend,
    source_factory: SourceFactory = _create_source,
    keyboard_factory: KeyboardFactory = KeyboardInput,
    gripper_factory: GripperFactory = _create_gripper_receiver,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    emit: Callable[[str], None] = print,
) -> None:
    """Validate live tracking, run teleoperation, and preserve optional debug telemetry."""
    args.validate()
    args = replace(
        args,
        tracker_to_robot_rpy_degrees=args.resolved_tracker_to_robot_rpy_degrees(),
    )
    gripper_calibrations = None
    if args.gripper_enabled:
        assert args.gripper_sender_ip is not None
        gripper_calibrations = load_umi_gripper_calibrations(
            args.resolved_umi_gripper_calibration_path(),
            expected_robot_id=args.robot_id,
            expected_sender_ip=args.gripper_sender_ip,
        )
    debug_recorder: DebugRecorder = (
        JsonlDebugRecorder(args.resolved_debug_log_path())
        if args.debug
        else NullDebugRecorder()
    )

    def logged_emit(message: str) -> None:
        emit(message)
        debug_recorder.event("message", message=message)

    outcome = "completed"
    try:
        if debug_recorder.path is not None:
            logged_emit(f"Debug telemetry: {debug_recorder.path}")
        debug_recorder.event(
            "run_start",
            args=args,
            lighthouse_config=args.resolved_config_path(),
            umi_gripper_calibration=args.resolved_umi_gripper_calibration_path()
            if args.gripper_enabled
            else None,
        )
        _run_session(
            args,
            backend_factory=backend_factory,
            source_factory=source_factory,
            keyboard_factory=keyboard_factory,
            gripper_factory=gripper_factory,
            gripper_calibrations=gripper_calibrations,
            monotonic=monotonic,
            sleep=sleep,
            emit=logged_emit,
            debug_recorder=debug_recorder,
        )
    except BaseException as error:
        outcome = "error"
        debug_recorder.event("fatal_error", error=error)
        raise
    finally:
        debug_recorder.event("run_end", outcome=outcome)
        debug_recorder.close()


def main() -> None:
    """CLI entry point."""
    try:
        run(tyro.cli(Args))
    except KeyboardInterrupt as error:
        raise SystemExit(130) from error
    except (ImportError, OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error


if __name__ == "__main__":
    main()

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
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Protocol

import numpy as np
import tyro
from numpy.typing import NDArray

from umi_teleop_fixed.calibration import cache_root
from umi_teleop_fixed.robot import (
    DEFAULT_START_POSITION,
    SIDES,
    ArmState,
    DualYamBackend,
    validate_dual_position,
)
from umi_teleop_fixed.se3 import (
    PoseMatrix,
    pose_matrix,
    rate_limit_pose,
    rotation_distance,
    scaled_relative_target,
    workspace_error,
)
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
    """Multiplier for rotation relative to the engage pose."""

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

    libsurvive_path: Path | None = None
    """Path to libsurvive.so when it is not installed in a standard path."""

    config_path: Path | None = None
    """Libsurvive config path; defaults to the user cache."""

    poser: str = "KalmanOnly"
    """Libsurvive poser plugin."""

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
            "rotation_scale": self.rotation_scale,
            "max_linear_speed_m_s": self.max_linear_speed_m_s,
            "max_angular_speed_rad_s": self.max_angular_speed_rad_s,
            "max_translation_from_start_m": self.max_translation_from_start_m,
            "max_rotation_from_start_rad": self.max_rotation_from_start_rad,
            "tracker_timeout_seconds": self.tracker_timeout_seconds,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        validate_dual_position(self.start_position)

    def resolved_config_path(self) -> Path:
        if self.config_path is not None:
            return self.config_path.expanduser()
        return cache_root() / "umi-teleop-fixed" / "libsurvive" / "config.json"


@dataclass
class _EngagedSide:
    tracker_pose: PoseMatrix
    robot_pose: PoseMatrix
    last_target: PoseMatrix
    last_joints: NDArray[np.float64]
    gripper_position: float


class TeleopController:
    """Pure engage-relative target generation with explicit commit semantics."""

    def __init__(self, args: Args) -> None:
        self._args = args
        self._engaged: dict[str, _EngagedSide] | None = None

    @property
    def engaged(self) -> bool:
        return self._engaged is not None

    def engage(
        self,
        states: Mapping[str, ArmState],
        samples: Mapping[str, TrackerSample],
    ) -> None:
        self._engaged = {}
        for side in SIDES:
            tracker = samples[side].pose
            self._engaged[side] = _EngagedSide(
                tracker_pose=pose_matrix(tracker.position, tracker.quaternion_wxyz),
                robot_pose=states[side].ee_pose.copy(),
                last_target=states[side].ee_pose.copy(),
                last_joints=states[side].joint_positions.copy(),
                gripper_position=states[side].gripper_position,
            )

    def disengage(self) -> None:
        self._engaged = None

    def propose(
        self,
        samples: Mapping[str, TrackerSample],
        *,
        dt: float,
    ) -> dict[str, PoseMatrix]:
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        targets = {}
        for side in SIDES:
            baseline = self._engaged[side]
            tracker = samples[side].pose
            requested = scaled_relative_target(
                baseline.tracker_pose,
                pose_matrix(tracker.position, tracker.quaternion_wxyz),
                baseline.robot_pose,
                translation_scale=self._args.translation_scale,
                rotation_scale=self._args.rotation_scale,
            )
            error = workspace_error(
                baseline.robot_pose,
                requested,
                max_translation_m=self._args.max_translation_from_start_m,
                max_rotation_rad=self._args.max_rotation_from_start_rad,
            )
            if error is not None:
                raise RuntimeError(f"{side} tracker target left the safe workspace: {error}")
            targets[side] = rate_limit_pose(
                baseline.last_target,
                requested,
                dt=dt,
                max_linear_speed_m_s=self._args.max_linear_speed_m_s,
                max_angular_speed_rad_s=self._args.max_angular_speed_rad_s,
            )
        return targets

    def seeds(self) -> dict[str, NDArray[np.float64]]:
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        return {side: self._engaged[side].last_joints.copy() for side in SIDES}

    def grippers(self) -> dict[str, float]:
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        return {side: self._engaged[side].gripper_position for side in SIDES}

    def commit(
        self,
        targets: Mapping[str, PoseMatrix],
        joint_targets: Mapping[str, NDArray[np.float64]],
    ) -> None:
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        for side in SIDES:
            self._engaged[side].last_target = targets[side].copy()
            self._engaged[side].last_joints = joint_targets[side].copy()

    def motion_summary(self) -> str:
        """Describe the latest engage-relative Cartesian target on both sides."""
        if self._engaged is None:
            raise RuntimeError("teleoperation is not engaged")
        summaries = []
        for side in SIDES:
            state = self._engaged[side]
            delta_mm = (state.last_target[:3, 3] - state.robot_pose[:3, 3]) * 1000.0
            rotation_deg = math.degrees(rotation_distance(state.robot_pose, state.last_target))
            vector = ",".join(f"{value:+.1f}" for value in delta_mm)
            summaries.append(f"{side} dxyz_mm=[{vector}] drot={rotation_deg:.1f}deg")
        return "  ".join(summaries)


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
    keys: KeySource,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    emit: Callable[[str], None] = print,
    max_cycles: int | None = None,
    initial_idle: bool = True,
) -> None:
    """Run an already-connected system; dependency injection keeps E2E tests deterministic."""
    controller = TeleopController(args)
    period = 1.0 / args.control_rate_hz
    next_tick = monotonic()
    status_started = next_tick
    successful_updates = 0
    cycles = 0
    if initial_idle:
        backend.idle()
    emit(f"Ready in {args.mode} mode. Keys: e engage, d disengage, r re-zero, q quit.")
    try:
        while max_cycles is None or cycles < max_cycles:
            cycles += 1
            quit_requested = False
            for key in keys.poll():
                if key == "q" or key == "\x03":
                    quit_requested = True
                elif key == "d" and controller.engaged:
                    controller.disengage()
                    backend.idle()
                    emit("DISENGAGED: both arms are in gravity compensation.")
                elif key in {"e", "r"}:
                    samples = trackers.snapshot(
                        now=monotonic(),
                        stale_after_seconds=args.tracker_timeout_seconds,
                    )
                    states = backend.read_states()
                    controller.engage(states, samples)
                    status_started = monotonic()
                    successful_updates = 0
                    action = "RE-ZEROED" if key == "r" else "ENGAGED"
                    emit(f"{action}: captured current tracker and robot poses ({args.mode}).")
            if quit_requested:
                return

            if controller.engaged:
                try:
                    samples = trackers.snapshot(
                        now=monotonic(),
                        stale_after_seconds=args.tracker_timeout_seconds,
                    )
                    backend.read_states()
                    targets = controller.propose(samples, dt=period)
                    joints = backend.solve_ik(targets, controller.seeds())
                    if args.mode == "active":
                        backend.command(joints, controller.grippers())
                    controller.commit(targets, joints)
                    successful_updates += 1
                except (RuntimeError, ValueError) as error:
                    controller.disengage()
                    backend.idle()
                    emit(f"SAFETY STOP: {error}. Both arms returned to gravity compensation.")

            now = monotonic()
            status_elapsed = now - status_started
            if controller.engaged and status_elapsed >= 1.0:
                rate = successful_updates / status_elapsed
                emit(
                    f"RUNNING {args.mode}: dual FK/IK healthy at {rate:.1f} Hz; "
                    f"{controller.motion_summary()}"
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


BackendFactory = Callable[[Args], DualYamBackend]
SourceFactory = Callable[[Args], PoseSource]
KeyboardFactory = Callable[[], KeyboardInput]


def _create_backend(args: Args) -> DualYamBackend:
    return DualYamBackend.open(
        robot_id=args.robot_id,
        left_adapter_serial=args.left_adapter_serial,
        right_adapter_serial=args.right_adapter_serial,
    )


def _create_source(args: Args) -> PoseSource:
    return LibsurviveSource(
        library_path=args.libsurvive_path,
        config_path=args.resolved_config_path(),
        poser=args.poser,
    )


def run(
    args: Args,
    *,
    backend_factory: BackendFactory = _create_backend,
    source_factory: SourceFactory = _create_source,
    keyboard_factory: KeyboardFactory = KeyboardInput,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    emit: Callable[[str], None] = print,
) -> None:
    """Validate live tracking before connecting both robot arms."""
    args.validate()
    source = source_factory(args)
    with TrackerReader(
        source,
        serials={"left": args.left_tracker_serial, "right": args.right_tracker_serial},
        monotonic=monotonic,
    ) as reader:
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
                    "Dry-run validated start_position; startup and teardown trajectories "
                    "will not send commands."
                )
                with keyboard_factory() as keyboard:
                    control_loop(
                        args,
                        backend=backend,
                        trackers=reader,
                        keys=keyboard,
                        monotonic=monotonic,
                        sleep=sleep,
                        emit=emit,
                    )
            else:
                backend.move_to_position(
                    args.start_position,
                    monotonic=monotonic,
                    sleep=sleep,
                    emit=emit,
                )
                try:
                    with keyboard_factory() as keyboard:
                        control_loop(
                            args,
                            backend=backend,
                            trackers=reader,
                            keys=keyboard,
                            monotonic=monotonic,
                            sleep=sleep,
                            emit=emit,
                            initial_idle=False,
                        )
                finally:
                    emit("Returning both arms to start_position before shutdown...")
                    backend.move_to_position(
                        args.start_position,
                        monotonic=monotonic,
                        sleep=sleep,
                        emit=emit,
                    )


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

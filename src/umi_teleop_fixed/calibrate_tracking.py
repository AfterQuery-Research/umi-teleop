"""Automatically calibrate Lighthouses and their axes relative to a robot."""

from __future__ import annotations

import math
import time
from collections import deque
from collections.abc import Callable, Mapping
from contextlib import closing
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Protocol

import numpy as np
import tyro
from numpy.typing import NDArray

from umi_teleop_fixed import calibrate_lighthouses
from umi_teleop_fixed.calibration import cache_root, validate_robot_id
from umi_teleop_fixed.lighthouses import (
    LighthouseStatus,
    save_new_lighthouse_calibration_id,
)
from umi_teleop_fixed.se3 import checked_rotation_matrix, matrix_to_rpy_degrees
from umi_teleop_fixed.teleop_frame import (
    TeleopFrameCalibration,
    save_teleop_frame,
    teleop_frame_path,
)
from umi_teleop_fixed.trackers import (
    LibsurviveSource,
    PoseSource,
    TrackerMonitor,
)

Position = NDArray[np.float64]


@dataclass(frozen=True)
class Args:
    """Calibrate Lighthouse geometry and tracker motion axes for one robot."""

    robot_id: str
    """Robot installation that owns this tracking calibration."""

    tracker_serial: str
    """One visible Vive tracker used for both calibration stages."""

    expected_lighthouses: int = 2
    """Number of powered Lighthouses that must be solved."""

    lighthouse_timeout_seconds: float = 60.0
    """Maximum time allowed for the Lighthouse scene solve."""

    lighthouse_settle_seconds: float = 5.0
    """Time a complete Lighthouse solution must remain stable."""

    gesture_timeout_seconds: float = 45.0
    """Maximum time allowed for each instructed movement or center return."""

    minimum_motion_m: float = 0.10
    """Minimum distance from center required to capture a direction."""

    stable_seconds: float = 0.6
    """How long the tracker must remain still before automatic capture."""

    stable_radius_m: float = 0.004
    """Maximum tracker spread while detecting a stable hold."""

    center_return_radius_m: float = 0.03
    """Maximum distance from center accepted between movements."""

    max_axis_error_degrees: float = 20.0
    """Maximum opposition, orthogonality, or fitted direction error."""

    min_tracker_rate_hz: float = 20.0
    """Minimum acceptable update rate for the selected tracker."""

    libsurvive_path: Path | None = None
    """Path to libsurvive.so when it is not installed in a standard path."""

    config_path: Path | None = None
    """Libsurvive config path; defaults to the user cache."""

    def resolved_config_path(self) -> Path:
        if self.config_path is not None:
            return self.config_path.expanduser()
        return cache_root() / "umi-teleop-fixed" / "libsurvive" / "config.json"

    def validate(self) -> None:
        validate_robot_id(self.robot_id)
        if not self.tracker_serial.strip():
            raise ValueError("tracker_serial must not be empty")
        positive = {
            "lighthouse_timeout_seconds": self.lighthouse_timeout_seconds,
            "lighthouse_settle_seconds": self.lighthouse_settle_seconds,
            "gesture_timeout_seconds": self.gesture_timeout_seconds,
            "minimum_motion_m": self.minimum_motion_m,
            "stable_seconds": self.stable_seconds,
            "stable_radius_m": self.stable_radius_m,
            "center_return_radius_m": self.center_return_radius_m,
            "max_axis_error_degrees": self.max_axis_error_degrees,
            "min_tracker_rate_hz": self.min_tracker_rate_hz,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.lighthouse_settle_seconds >= self.lighthouse_timeout_seconds:
            raise ValueError(
                "lighthouse_settle_seconds must be shorter than lighthouse_timeout_seconds"
            )
        if self.stable_seconds >= self.gesture_timeout_seconds:
            raise ValueError("stable_seconds must be shorter than gesture_timeout_seconds")
        if self.center_return_radius_m >= self.minimum_motion_m:
            raise ValueError("center_return_radius_m must be smaller than minimum_motion_m")
        if self.stable_radius_m >= self.center_return_radius_m:
            raise ValueError("stable_radius_m must be smaller than center_return_radius_m")
        if self.max_axis_error_degrees >= 45.0:
            raise ValueError("max_axis_error_degrees must be less than 45")
        calibrate_lighthouses.Args(
            tracker_serial=self.tracker_serial,
            expected_lighthouses=self.expected_lighthouses,
        ).validate()


@dataclass(frozen=True)
class FrameSolution:
    """Solved rotation and diagnostics for one guided directional capture."""

    rotation: NDArray[np.float64]
    rpy_degrees: tuple[float, float, float]
    direction_errors_degrees: tuple[float, float, float]
    opposition_errors_degrees: tuple[float, ...]
    orthogonality_errors_degrees: tuple[float, float, float]


@dataclass(frozen=True)
class TrackingCalibrationResult:
    """Artifacts produced by a completed combined calibration."""

    lighthouse_status: LighthouseStatus
    frame_path: Path
    frame: FrameSolution


class LighthouseRunner(Protocol):
    def __call__(
        self,
        args: calibrate_lighthouses.Args,
        **kwargs: object,
    ) -> LighthouseStatus: ...


PoseSourceFactory = Callable[[Args], PoseSource]


def _create_pose_source(args: Args) -> PoseSource:
    return LibsurviveSource(
        library_path=args.libsurvive_path,
        config_path=args.resolved_config_path(),
        poser="MPFIT",
        disable_calibration=True,
    )


class _StablePositionWindow:
    def __init__(self, *, duration_seconds: float, radius_m: float) -> None:
        self._duration_seconds = duration_seconds
        self._radius_m = radius_m
        self._samples: deque[tuple[float, Position]] = deque()

    def observe(self, *, now: float, position: Position) -> Position | None:
        self._samples.append((now, position.copy()))
        oldest_allowed = now - self._duration_seconds
        while self._samples and self._samples[0][0] < oldest_allowed - 1e-9:
            self._samples.popleft()
        if len(self._samples) < 2:
            return None
        if now - self._samples[0][0] < self._duration_seconds * 0.95:
            return None
        positions = np.stack([sample for _, sample in self._samples])
        center = np.mean(positions, axis=0)
        spread = float(np.max(np.linalg.norm(positions - center, axis=1)))
        return center if spread <= self._radius_m else None


def _wait_for_stable_position(
    source: PoseSource,
    *,
    tracker_serial: str,
    monitor: TrackerMonitor,
    monotonic: Callable[[], float],
    timeout_seconds: float,
    stable_seconds: float,
    stable_radius_m: float,
    accept: Callable[[Position], bool],
    timeout_description: str,
    emit: Callable[[str], None] | None = None,
    reached_message: str | None = None,
    lost_message: str | None = None,
) -> Position:
    detector = _StablePositionWindow(
        duration_seconds=stable_seconds,
        radius_m=stable_radius_m,
    )
    tracker_key = tracker_serial.casefold()
    deadline = monotonic() + timeout_seconds
    was_accepted: bool | None = None
    while monotonic() < deadline:
        pose = source.next_pose()
        now = monotonic()
        if pose is None or pose.serial.casefold() != tracker_key:
            continue
        if not monitor.observe(pose, received_at=now):
            continue
        position = np.asarray(pose.position, dtype=np.float64)
        accepted = accept(position)
        if accepted and was_accepted is not True and emit is not None and reached_message:
            emit(reached_message)
        elif not accepted and was_accepted is True and emit is not None and lost_message:
            emit(lost_message)
        was_accepted = accepted
        stable = detector.observe(now=now, position=position)
        if stable is not None and accept(stable):
            return stable
    raise RuntimeError(f"timed out waiting for {timeout_description}")


def _unit(vector: Position, *, name: str) -> Position:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        raise ValueError(f"{name} direction has zero length")
    return vector / norm


def _angle_degrees(first: Position, second: Position) -> float:
    cosine = float(np.clip(np.dot(_unit(first, name="first"), _unit(second, name="second")), -1, 1))
    return math.degrees(math.acos(cosine))


def solve_frame(
    center: Position,
    captures: Mapping[str, Position],
    *,
    max_axis_error_degrees: float,
) -> FrameSolution:
    """Fit a proper Lighthouse-world to robot-base rotation from five positions."""
    required = ("forward", "backward", "left", "right", "up")
    missing = [name for name in required if name not in captures]
    if missing:
        raise ValueError(f"missing direction captures: {', '.join(missing)}")
    center = np.asarray(center, dtype=np.float64)
    if center.shape != (3,) or not np.all(np.isfinite(center)):
        raise ValueError("center must be a finite 3-D position")
    offsets = {name: np.asarray(captures[name], dtype=np.float64) - center for name in required}
    if any(value.shape != (3,) or not np.all(np.isfinite(value)) for value in offsets.values()):
        raise ValueError("direction captures must be finite 3-D positions")

    pairs = (
        ("forward", "backward"),
        ("left", "right"),
    )
    opposition_errors = tuple(
        _angle_degrees(offsets[positive], -offsets[negative]) for positive, negative in pairs
    )
    opposition_limit = 2.0 * max_axis_error_degrees
    if max(opposition_errors) > opposition_limit:
        raise RuntimeError(
            "opposite movements were inconsistent: "
            + ", ".join(f"{value:.1f}deg" for value in opposition_errors)
            + f" (limit {opposition_limit:.1f}deg)"
        )

    measured_axes = (
        _unit(captures["forward"] - captures["backward"], name="forward"),
        _unit(captures["left"] - captures["right"], name="left"),
        _unit(captures["up"] - center, name="up"),
    )
    orthogonality_errors = tuple(
        abs(90.0 - _angle_degrees(measured_axes[first], measured_axes[second]))
        for first, second in ((0, 1), (0, 2), (1, 2))
    )
    if max(orthogonality_errors) > max_axis_error_degrees:
        raise RuntimeError(
            "movement axes were not sufficiently perpendicular: "
            + ", ".join(f"{value:.1f}deg" for value in orthogonality_errors)
        )

    source_directions = np.stack([_unit(offsets[name], name=name) for name in required])
    target_directions = np.asarray(
        (
            (1.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, -1.0, 0.0),
            (0.0, 0.0, 1.0),
        )
    )
    u, _singular_values, vt = np.linalg.svd(source_directions.T @ target_directions)
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(vt.T @ u.T)
    rotation = checked_rotation_matrix(vt.T @ correction @ u.T)
    mapped = (rotation @ source_directions.T).T
    individual_errors = tuple(
        _angle_degrees(mapped[index], target_directions[index]) for index in range(5)
    )
    direction_errors = (
        max(individual_errors[0], individual_errors[1]),
        max(individual_errors[2], individual_errors[3]),
        individual_errors[4],
    )
    if max(direction_errors) > max_axis_error_degrees:
        raise RuntimeError(
            "fitted tracker directions exceeded the error limit: "
            + ", ".join(f"{value:.1f}deg" for value in direction_errors)
        )
    return FrameSolution(
        rotation=rotation,
        rpy_degrees=matrix_to_rpy_degrees(rotation),
        direction_errors_degrees=direction_errors,
        opposition_errors_degrees=opposition_errors,
        orthogonality_errors_degrees=orthogonality_errors,
    )


_DIRECTIONS = (
    (
        "forward",
        "ROBOT FORWARD (+X)",
        "Move the tracker straight FORWARD, away from your body and the robot base.",
    ),
    (
        "backward",
        "ROBOT BACKWARD (-X)",
        "Move the tracker straight BACKWARD, toward your body and the robot base.",
    ),
    (
        "left",
        "ROBOT LEFT (+Y)",
        "Move the tracker to YOUR LEFT.",
    ),
    (
        "right",
        "ROBOT RIGHT (-Y)",
        "Move the tracker to YOUR RIGHT.",
    ),
    ("up", "UP (+Z)", "Move the tracker straight UP."),
)

_MAX_DIRECTION_ATTEMPTS = 3


def _capture_direction_attempt(
    args: Args,
    source: PoseSource,
    *,
    tracker_serial: str,
    monitor: TrackerMonitor,
    monotonic: Callable[[], float],
    emit: Callable[[str], None],
) -> tuple[Position, dict[str, Position]]:
    emit("")
    emit("OPERATOR POSITION: Stand behind the robot and face the same direction as the robot.")
    emit("From there, your forward, left, and right match the robot's directions.")
    emit("Hold the tracker at a comfortable CENTER in front of your body, then keep still.")
    center = _wait_for_stable_position(
        source,
        tracker_serial=tracker_serial,
        monitor=monitor,
        monotonic=monotonic,
        timeout_seconds=args.gesture_timeout_seconds,
        stable_seconds=args.stable_seconds,
        stable_radius_m=args.stable_radius_m,
        accept=lambda _position: True,
        timeout_description="a stable center position",
    )
    emit("CENTER captured automatically.")
    captures: dict[str, Position] = {}
    for step, (name, label, instruction) in enumerate(_DIRECTIONS, start=1):
        emit("")
        emit(f"STEP {step}/{len(_DIRECTIONS)} — {label}")
        emit(instruction)
        emit("Keep moving slowly until the program tells you to stop.")
        endpoint = _wait_for_stable_position(
            source,
            tracker_serial=tracker_serial,
            monitor=monitor,
            monotonic=monotonic,
            timeout_seconds=args.gesture_timeout_seconds,
            stable_seconds=args.stable_seconds,
            stable_radius_m=args.stable_radius_m,
            accept=lambda position: (
                float(np.linalg.norm(position - center)) >= args.minimum_motion_m
            ),
            timeout_description=f"the {label.lower()} movement",
            emit=emit,
            reached_message=(
                "Distance reached. STOP and HOLD the tracker still; capture is automatic."
            ),
            lost_message="Move farther in the instructed direction, then hold still.",
        )
        captures[name] = endpoint
        emit(f"{label} captured.")
        emit("Now move the tracker back to CENTER; the program will tell you when to stop.")
        _wait_for_stable_position(
            source,
            tracker_serial=tracker_serial,
            monitor=monitor,
            monotonic=monotonic,
            timeout_seconds=args.gesture_timeout_seconds,
            stable_seconds=args.stable_seconds,
            stable_radius_m=args.stable_radius_m,
            accept=lambda position: (
                float(np.linalg.norm(position - center)) <= args.center_return_radius_m
            ),
            timeout_description="the tracker to return to center",
            emit=emit,
            reached_message="CENTER reached. STOP and HOLD the tracker still.",
            lost_message="You left CENTER. Move back toward the original center position.",
        )
        emit("CENTER captured. Advancing to the next instruction.")
    return center, captures


def run(
    args: Args,
    *,
    lighthouse_runner: LighthouseRunner = calibrate_lighthouses.run,
    pose_source_factory: PoseSourceFactory = _create_pose_source,
    monotonic: Callable[[], float] = time.monotonic,
    emit: Callable[[str], None] = print,
) -> TrackingCalibrationResult:
    """Run both calibration stages and save a paired room-to-robot mapping."""
    args.validate()
    emit("This is a fully guided calibration; do not measure distance or count time.")
    emit("Follow only the current instruction and wait for the program to advance.")
    emit("First, keep the tracker completely still for the Lighthouse solve.")
    config_path = args.resolved_config_path()
    lighthouse_args = calibrate_lighthouses.Args(
        tracker_serial=args.tracker_serial,
        timeout_seconds=args.lighthouse_timeout_seconds,
        settle_seconds=args.lighthouse_settle_seconds,
        min_tracker_rate_hz=args.min_tracker_rate_hz,
        expected_lighthouses=args.expected_lighthouses,
        libsurvive_path=args.libsurvive_path,
        config_path=config_path,
    )
    status = lighthouse_runner(lighthouse_args, monotonic=monotonic, emit=emit)
    calibration_id = save_new_lighthouse_calibration_id(config_path)
    emit("Lighthouse solve complete. The previous tracker-to-robot mapping is now stale.")

    tracker_serial = args.tracker_serial.strip()
    monitor = TrackerMonitor(left_serial=None, right_serial=None)
    solution: FrameSolution | None = None
    with closing(pose_source_factory(args)) as source:
        for attempt in range(1, _MAX_DIRECTION_ATTEMPTS + 1):
            if attempt > 1:
                emit("")
                emit(
                    f"Repeating the guided directions (attempt {attempt}/"
                    f"{_MAX_DIRECTION_ATTEMPTS}); the Lighthouses remain calibrated."
                )
            center, captures = _capture_direction_attempt(
                args,
                source,
                tracker_serial=tracker_serial,
                monitor=monitor,
                monotonic=monotonic,
                emit=emit,
            )
            try:
                solution = solve_frame(
                    center,
                    captures,
                    max_axis_error_degrees=args.max_axis_error_degrees,
                )
            except RuntimeError as error:
                if attempt == _MAX_DIRECTION_ATTEMPTS:
                    raise RuntimeError(
                        f"directional calibration failed after {attempt} attempts: {error}"
                    ) from error
                emit(f"The directions were not consistent enough: {error}")
                emit("Keep the tracker at CENTER; the program will guide a retry automatically.")
                continue
            break

        finished = monotonic()
        errors = monitor.health_errors(
            now=finished,
            expected_count=1,
            stale_after_seconds=0.25,
            min_rate_hz=args.min_tracker_rate_hz,
        )
        if errors:
            raise RuntimeError("Tracker validation failed:\n" + "\n".join(errors))

    if solution is None:
        raise RuntimeError("directional calibration did not produce a frame")
    frame_path = teleop_frame_path(args.robot_id)
    calibration = TeleopFrameCalibration(
        rpy_degrees=solution.rpy_degrees,
        calibrated_at_utc=datetime.now(UTC).isoformat(),
        source="automatic five-direction tracker calibration",
        lighthouse_calibration_id=calibration_id,
        direction_errors_degrees=solution.direction_errors_degrees,
    )
    save_teleop_frame(frame_path, robot_id=args.robot_id, calibration=calibration)
    formatted_rpy = " ".join(f"{value:+.2f}" for value in solution.rpy_degrees)
    formatted_errors = " ".join(f"{value:.1f}" for value in solution.direction_errors_degrees)
    emit(f"Saved paired tracking calibration to {frame_path}")
    emit(f"Lighthouse-to-robot RPY degrees: [{formatted_rpy}]")
    emit(f"Direction errors XYZ degrees: [{formatted_errors}]")
    emit("Calibration complete. Active teleoperation may now use this robot ID.")
    return TrackingCalibrationResult(status, frame_path, solution)


def main() -> None:
    """CLI entry point."""
    try:
        run(tyro.cli(Args))
    except KeyboardInterrupt as error:
        raise SystemExit(130) from error
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error


if __name__ == "__main__":
    main()

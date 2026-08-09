"""Calibrate a libsurvive lighthouse scene without connecting to a robot."""

from __future__ import annotations

import math
import os
import re
import tempfile
import time
from collections.abc import Callable
from contextlib import closing, suppress
from dataclasses import dataclass
from pathlib import Path

import tyro

from umi_teleop_fixed.calibration import cache_root
from umi_teleop_fixed.lighthouses import (
    MIN_LIGHTHOUSES,
    LighthouseStatus,
    load_lighthouse_status,
    require_lighthouse_calibration,
)
from umi_teleop_fixed.trackers import (
    LibsurviveSource,
    PoseSource,
    TrackerMonitor,
)

# The installed libsurvive MPFIT scene solver has a 16-element variance buffer
# with two entries per lighthouse. Limit calibration to its safe range.
MAX_CALIBRATION_LIGHTHOUSES = 8


@dataclass(frozen=True)
class Args:
    """Solve and save a stationary SteamVR lighthouse scene."""

    tracker_serial: str
    """Vive tracker serial used to validate the calibration, such as LHR-1234ABCD."""

    expected_lighthouses: int
    """Number of powered lighthouses that must be solved."""

    timeout_seconds: float = 30.0
    """Maximum time allowed to find and solve all expected lighthouses."""

    settle_seconds: float = 5.0
    """Time the complete solution must remain available before it is saved."""

    min_tracker_rate_hz: float = 20.0
    """Minimum acceptable pose update rate for the selected tracker."""

    libsurvive_path: Path | None = None
    """Path to libsurvive.so when it is not installed in a standard path."""

    config_path: Path | None = None
    """Saved libsurvive config; defaults to the user cache."""

    def validate(self) -> None:
        if not self.tracker_serial.strip():
            raise ValueError("tracker_serial must not be empty")
        for name, value in (
            ("timeout_seconds", self.timeout_seconds),
            ("settle_seconds", self.settle_seconds),
            ("min_tracker_rate_hz", self.min_tracker_rate_hz),
        ):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.settle_seconds >= self.timeout_seconds:
            raise ValueError("settle_seconds must be shorter than timeout_seconds")
        if not MIN_LIGHTHOUSES <= self.expected_lighthouses <= MAX_CALIBRATION_LIGHTHOUSES:
            raise ValueError(
                f"expected_lighthouses must be between {MIN_LIGHTHOUSES} "
                f"and {MAX_CALIBRATION_LIGHTHOUSES}"
            )

    def resolved_config_path(self) -> Path:
        if self.config_path is not None:
            return self.config_path.expanduser()
        return cache_root() / "umi-teleop-fixed" / "libsurvive" / "config.json"


SourceFactory = Callable[[Args, Path], PoseSource]


def _prepare_candidate(config_path: Path) -> Path:
    """Create an uncalibrated candidate while leaving the saved config untouched."""
    config_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".lighthouse-calibration.",
        suffix=".json",
        dir=config_path.parent,
    )
    candidate = Path(temporary_name)
    try:
        contents = config_path.read_text(encoding="utf-8") if config_path.exists() else ""
        contents = re.sub(
            r'"PositionSet"\s*:\s*"[01]"',
            '"PositionSet":"0"',
            contents,
        )
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            output.write(contents)
            output.flush()
            os.fsync(output.fileno())
        candidate.chmod(0o600)
    except BaseException:
        with suppress(OSError):
            os.close(descriptor)
        candidate.unlink(missing_ok=True)
        raise
    return candidate


def _create_source(args: Args, candidate: Path) -> PoseSource:
    return LibsurviveSource(
        library_path=args.libsurvive_path,
        config_path=candidate,
        poser="MPFIT",
        force_calibration=True,
        lighthouse_count=args.expected_lighthouses,
    )


def _ready_status(path: Path, *, expected_count: int) -> LighthouseStatus | None:
    try:
        status = load_lighthouse_status(path)
    except (RuntimeError, ValueError):
        return None
    return status if status.ready(expected_count=expected_count) else None


def run(
    args: Args,
    *,
    source_factory: SourceFactory = _create_source,
    monotonic: Callable[[], float] = time.monotonic,
    emit: Callable[[str], None] = print,
) -> LighthouseStatus:
    """Build a fresh candidate config and install it only after validation."""
    args.validate()
    config_path = args.resolved_config_path()
    candidate = _prepare_candidate(config_path)
    monitor = TrackerMonitor(left_serial=None, right_serial=None)
    tracker_serial = args.tracker_serial.strip()
    tracker_key = tracker_serial.casefold()
    started = monotonic()
    deadline = started + args.timeout_seconds
    solved_at: float | None = None

    emit(
        f"Keep tracker {tracker_serial} stationary and visible to all "
        f"{args.expected_lighthouses} lighthouses."
    )
    emit(
        f"Solving {args.expected_lighthouses} lighthouse poses with MPFIT "
        f"(timeout {args.timeout_seconds:g}s)..."
    )
    try:
        with closing(source_factory(args, candidate)) as source:
            while True:
                if monotonic() >= deadline:
                    raise RuntimeError(
                        "Lighthouse calibration timed out; the previous calibration was preserved"
                    )
                pose = source.next_pose()
                now = monotonic()
                if pose is not None and pose.serial.casefold() == tracker_key:
                    monitor.observe(pose, received_at=now)

                status = _ready_status(
                    candidate,
                    expected_count=args.expected_lighthouses,
                )
                if status is None:
                    solved_at = None
                    continue
                if solved_at is None:
                    solved_at = now
                    emit(
                        f"Fresh solution found for {status.configured_count}/"
                        f"{len(status.lighthouses)} lighthouses; settling..."
                    )
                elif now - solved_at >= args.settle_seconds:
                    break

        status = require_lighthouse_calibration(
            candidate,
            expected_count=args.expected_lighthouses,
        )
        finished = monotonic()
        errors = monitor.health_errors(
            now=finished,
            expected_count=1,
            stale_after_seconds=0.25,
            min_rate_hz=args.min_tracker_rate_hz,
        )
        if errors:
            raise RuntimeError(
                "Tracker validation failed; the previous calibration was preserved:\n"
                + "\n".join(errors)
            )

        candidate.chmod(0o600)
        os.replace(candidate, config_path)
        emit(f"Saved lighthouse calibration to {config_path}")
        for lighthouse in status.lighthouses:
            position = " ".join(f"{value:+.3f}" for value in lighthouse.pose[:3])
            emit(
                f"lighthouse {lighthouse.index}: id={lighthouse.base_station_id} "
                f"position_m=[{position}]"
            )
        emit(
            f"Calibration complete: {status.configured_count}/"
            f"{len(status.lighthouses)} lighthouse poses are ready."
        )
        return status
    finally:
        candidate.unlink(missing_ok=True)


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

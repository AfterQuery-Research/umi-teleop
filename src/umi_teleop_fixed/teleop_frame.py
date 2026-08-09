"""Robot-specific room-to-robot frame calibration."""

from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import asdict, dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import tyro

from umi_teleop_fixed.calibration import cache_root, validate_robot_id

SCHEMA_VERSION = 2
SUPPORTED_SCHEMA_VERSIONS = {1, SCHEMA_VERSION}


def _validated_rpy(values: object) -> tuple[float, float, float]:
    if isinstance(values, str | bytes | dict):
        raise ValueError("rpy_degrees must contain roll, pitch, and yaw")
    try:
        rpy = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError("rpy_degrees must contain roll, pitch, and yaw") from error
    if len(rpy) != 3 or not all(math.isfinite(value) for value in rpy):
        raise ValueError("rpy_degrees must contain three finite values")
    return rpy


def _validated_direction_errors(values: object) -> tuple[float, float, float] | None:
    if values is None:
        return None
    if isinstance(values, str | bytes | dict):
        raise ValueError("direction_errors_degrees must contain x, y, and z errors")
    try:
        errors = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError("direction_errors_degrees must contain x, y, and z errors") from error
    if len(errors) != 3 or not all(math.isfinite(value) and value >= 0 for value in errors):
        raise ValueError("direction_errors_degrees must contain three nonnegative finite values")
    return errors


def _validated_calibration_id(value: object) -> str | None:
    if value is None:
        return None
    calibration_id = str(value).strip().casefold()
    if len(calibration_id) != 64 or any(
        character not in "0123456789abcdef" for character in calibration_id
    ):
        raise ValueError("lighthouse_calibration_id must be a 64-character hexadecimal token")
    return calibration_id


@dataclass(frozen=True)
class TeleopFrameCalibration:
    """Fixed rotation from lighthouse-world axes to robot-base axes."""

    rpy_degrees: tuple[float, float, float]
    calibrated_at_utc: str
    source: str
    lighthouse_calibration_id: str | None = None
    direction_errors_degrees: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "rpy_degrees", _validated_rpy(self.rpy_degrees))
        object.__setattr__(
            self,
            "lighthouse_calibration_id",
            _validated_calibration_id(self.lighthouse_calibration_id),
        )
        object.__setattr__(
            self,
            "direction_errors_degrees",
            _validated_direction_errors(self.direction_errors_degrees),
        )
        if not self.calibrated_at_utc.strip():
            raise ValueError("calibrated_at_utc must not be empty")
        if not self.source.strip():
            raise ValueError("source must not be empty")

    @classmethod
    def from_payload(cls, payload: object) -> TeleopFrameCalibration:
        if not isinstance(payload, dict):
            raise ValueError("teleop frame calibration must be an object")
        try:
            return cls(
                rpy_degrees=_validated_rpy(payload["rpy_degrees"]),
                calibrated_at_utc=str(payload["calibrated_at_utc"]),
                source=str(payload["source"]),
                lighthouse_calibration_id=_validated_calibration_id(
                    payload.get("lighthouse_calibration_id")
                ),
                direction_errors_degrees=_validated_direction_errors(
                    payload.get("direction_errors_degrees")
                ),
            )
        except KeyError as error:
            raise ValueError(f"missing teleop frame field: {error.args[0]}") from error


def teleop_frame_path(robot_id: str, *, root: Path | None = None) -> Path:
    """Return the cached frame-calibration path for one robot installation."""
    identity = validate_robot_id(robot_id)
    base = cache_root() if root is None else root
    return base / "umi-teleop-fixed" / "calibration" / "teleop_frames" / f"{identity}.json"


def load_teleop_frame(
    path: Path,
    *,
    expected_robot_id: str,
    expected_lighthouse_calibration_id: str | None = None,
) -> TeleopFrameCalibration:
    """Load a required robot-specific frame calibration."""
    identity = validate_robot_id(expected_robot_id)
    if not path.exists():
        raise RuntimeError(
            f"missing teleop frame calibration for {identity!r} at {path}; run "
            f"umi-calibrate-tracking --robot-id {identity} --tracker-serial TRACKER_SERIAL"
        )
    try:
        payload: Any = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Could not read teleop frame calibration {path}: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError("teleop frame calibration file must contain an object")
    if payload.get("schema_version") not in SUPPORTED_SCHEMA_VERSIONS:
        raise ValueError(f"unsupported teleop frame schema in {path}")
    if payload.get("robot_id") != identity:
        raise ValueError(f"teleop frame robot_id does not match {identity!r}")
    calibration = TeleopFrameCalibration.from_payload(payload.get("frame"))
    expected = _validated_calibration_id(expected_lighthouse_calibration_id)
    if expected is not None and calibration.lighthouse_calibration_id != expected:
        raise RuntimeError(
            "teleop frame calibration does not match the current Lighthouse scene; "
            "run 'uv run umi-calibrate-tracking' before active teleoperation"
        )
    return calibration


def save_teleop_frame(
    path: Path,
    *,
    robot_id: str,
    calibration: TeleopFrameCalibration,
) -> None:
    """Atomically save one robot-specific frame calibration."""
    identity = validate_robot_id(robot_id)
    payload = {
        "schema_version": SCHEMA_VERSION,
        "robot_id": identity,
        "frame": asdict(calibration),
    }
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as output:
            json.dump(payload, output, indent=2)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        temporary_path.chmod(0o600)
        os.replace(temporary_path, path)
    finally:
        temporary_path.unlink(missing_ok=True)


@dataclass(frozen=True)
class Args:
    """Save a measured lighthouse-world to robot-base rotation."""

    robot_id: str
    """Robot installation that owns this room-frame calibration."""

    rpy_degrees: tuple[float, float, float]
    """Extrinsic XYZ roll, pitch, yaw from lighthouse world to robot base."""

    source: str = "directional experiment"
    """Short note describing how the transform was measured."""

    def validate(self) -> None:
        validate_robot_id(self.robot_id)
        _validated_rpy(self.rpy_degrees)
        if not self.source.strip():
            raise ValueError("source must not be empty")


def run(args: Args) -> Path:
    """Validate and save the frame without connecting to trackers or a robot."""
    args.validate()
    path = teleop_frame_path(args.robot_id)
    calibration = TeleopFrameCalibration(
        rpy_degrees=args.rpy_degrees,
        calibrated_at_utc=datetime.now(UTC).isoformat(),
        source=args.source,
    )
    save_teleop_frame(path, robot_id=args.robot_id, calibration=calibration)
    print(f"Saved teleop frame {calibration.rpy_degrees} to {path}")
    return path


def main() -> None:
    """CLI entry point."""
    try:
        run(tyro.cli(Args))
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error


if __name__ == "__main__":
    main()

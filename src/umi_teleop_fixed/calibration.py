"""Standalone storage for hardware calibration data."""

from __future__ import annotations

import json
import math
import os
import re
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

SCHEMA_VERSION = 1
UMI_GRIPPER_SCHEMA_VERSION = 1
VALID_SIDES = frozenset({"left", "right"})
_ROBOT_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


def _validated_limits(values: object) -> tuple[float, float]:
    if isinstance(values, str | bytes | dict):
        raise ValueError("gripper_limits must contain [closed, open]")
    try:
        endpoints = tuple(values)
    except TypeError as error:
        raise ValueError("gripper_limits must contain [closed, open]") from error
    if len(endpoints) != 2:
        raise ValueError("gripper_limits must contain [closed, open]")
    try:
        closed, open_ = (float(value) for value in endpoints)
    except (TypeError, ValueError) as error:
        raise ValueError("gripper_limits must be numeric") from error
    if not math.isfinite(closed) or not math.isfinite(open_) or closed == open_:
        raise ValueError("gripper_limits must contain two distinct finite values")
    return closed, open_


def validate_robot_id(robot_id: str) -> str:
    """Validate an ID before using it as a cache filename."""
    value = robot_id.strip()
    if not _ROBOT_ID.fullmatch(value):
        raise ValueError("robot_id may contain only letters, numbers, '.', '_', and '-'")
    return value


@dataclass(frozen=True)
class GripperCalibration:
    """Raw i2rt gripper endpoints and the hardware identity that produced them."""

    adapter_serial: str
    gripper_type: str
    gripper_limits: tuple[float, float]
    calibrated_at_utc: str

    def __post_init__(self) -> None:
        if not self.adapter_serial.strip():
            raise ValueError("adapter_serial must not be empty")
        if not self.gripper_type.strip():
            raise ValueError("gripper_type must not be empty")
        object.__setattr__(self, "gripper_limits", _validated_limits(self.gripper_limits))
        if not self.calibrated_at_utc.strip():
            raise ValueError("calibrated_at_utc must not be empty")

    @classmethod
    def from_payload(cls, payload: object) -> GripperCalibration:
        if not isinstance(payload, dict):
            raise ValueError("gripper calibration entry must be an object")
        try:
            return cls(
                adapter_serial=str(payload["adapter_serial"]),
                gripper_type=str(payload["gripper_type"]),
                gripper_limits=_validated_limits(payload["gripper_limits"]),
                calibrated_at_utc=str(payload["calibrated_at_utc"]),
            )
        except KeyError as error:
            raise ValueError(f"missing calibration field: {error.args[0]}") from error


@dataclass(frozen=True)
class UmiGripperCalibration:
    """Physical UMI jaw endpoints used to normalize raw camera widths."""

    sender_ip: str
    closed_width_mm: float
    open_width_mm: float
    calibrated_at_utc: str

    def __post_init__(self) -> None:
        if not self.sender_ip.strip():
            raise ValueError("sender_ip must not be empty")
        values = (float(self.closed_width_mm), float(self.open_width_mm))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("UMI gripper widths must be finite")
        if values[0] <= 0 or values[1] <= values[0]:
            raise ValueError("UMI open width must be greater than its positive closed width")
        object.__setattr__(self, "closed_width_mm", values[0])
        object.__setattr__(self, "open_width_mm", values[1])
        if not self.calibrated_at_utc.strip():
            raise ValueError("calibrated_at_utc must not be empty")

    @property
    def span_mm(self) -> float:
        return self.open_width_mm - self.closed_width_mm

    @classmethod
    def from_payload(cls, payload: object) -> UmiGripperCalibration:
        if not isinstance(payload, dict):
            raise ValueError("UMI gripper calibration entry must be an object")
        try:
            return cls(
                sender_ip=str(payload["sender_ip"]),
                closed_width_mm=float(payload["closed_width_mm"]),
                open_width_mm=float(payload["open_width_mm"]),
                calibrated_at_utc=str(payload["calibrated_at_utc"]),
            )
        except KeyError as error:
            raise ValueError(f"missing UMI calibration field: {error.args[0]}") from error
        except (TypeError, ValueError) as error:
            raise ValueError(f"invalid UMI calibration field: {error}") from error


def cache_root() -> Path:
    """Return the platform cache root, respecting XDG_CACHE_HOME."""
    configured = os.environ.get("XDG_CACHE_HOME")
    return Path(configured).expanduser() if configured else Path.home() / ".cache"


def calibration_path(robot_id: str, *, root: Path | None = None) -> Path:
    """Return the standalone gripper-calibration file for one robot."""
    identity = validate_robot_id(robot_id)
    base = cache_root() if root is None else root
    return base / "umi-teleop-fixed" / "calibration" / "grippers" / f"{identity}.json"


def umi_gripper_calibration_path(robot_id: str, *, root: Path | None = None) -> Path:
    """Return the paired UMI endpoint-calibration file for one robot setup."""
    identity = validate_robot_id(robot_id)
    base = cache_root() if root is None else root
    return base / "umi-teleop-fixed" / "calibration" / "umi_grippers" / f"{identity}.json"


def load_gripper_calibrations(
    path: Path, *, expected_robot_id: str
) -> dict[str, GripperCalibration]:
    """Load and strictly validate an existing calibration file."""
    if not path.exists():
        return {}
    try:
        payload: Any = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Could not read calibration file {path}: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError("calibration file must contain an object")
    if payload.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"unsupported calibration schema in {path}")
    if payload.get("robot_id") != expected_robot_id:
        raise ValueError(f"calibration file robot_id does not match {expected_robot_id!r}")
    grippers = payload.get("grippers")
    if not isinstance(grippers, dict):
        raise ValueError("calibration file must contain a grippers object")
    unknown_sides = set(grippers) - VALID_SIDES
    if unknown_sides:
        raise ValueError(f"unknown calibration sides: {sorted(unknown_sides)}")
    return {side: GripperCalibration.from_payload(value) for side, value in grippers.items()}


def save_gripper_calibration(
    path: Path,
    *,
    robot_id: str,
    side: str,
    calibration: GripperCalibration,
) -> None:
    """Atomically update one side while preserving the other side."""
    identity = validate_robot_id(robot_id)
    if side not in VALID_SIDES:
        raise ValueError(f"side must be one of {sorted(VALID_SIDES)}")
    calibrations = load_gripper_calibrations(path, expected_robot_id=identity)
    calibrations[side] = calibration
    payload = {
        "schema_version": SCHEMA_VERSION,
        "robot_id": identity,
        "grippers": {name: asdict(value) for name, value in sorted(calibrations.items())},
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


def load_umi_gripper_calibrations(
    path: Path,
    *,
    expected_robot_id: str,
    expected_sender_ip: str,
) -> dict[str, UmiGripperCalibration]:
    """Load complete left/right UMI endpoints and verify their source."""
    if not path.exists():
        raise RuntimeError(
            f"missing UMI gripper calibration {path}; run umi-calibrate-umi-grippers"
        )
    try:
        payload: Any = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Could not read UMI calibration file {path}: {error}") from error
    if not isinstance(payload, dict):
        raise ValueError("UMI calibration file must contain an object")
    if payload.get("schema_version") != UMI_GRIPPER_SCHEMA_VERSION:
        raise ValueError(f"unsupported UMI gripper calibration schema in {path}")
    if payload.get("robot_id") != expected_robot_id:
        raise ValueError(f"UMI calibration robot_id does not match {expected_robot_id!r}")
    grippers = payload.get("grippers")
    if not isinstance(grippers, dict) or set(grippers) != VALID_SIDES:
        raise ValueError("UMI calibration must contain exactly left and right grippers")
    result = {
        side: UmiGripperCalibration.from_payload(value) for side, value in grippers.items()
    }
    mismatched = [side for side, value in result.items() if value.sender_ip != expected_sender_ip]
    if mismatched:
        raise ValueError(
            f"UMI calibration sender IP does not match {expected_sender_ip} for "
            f"{', '.join(sorted(mismatched))}"
        )
    return result


def save_umi_gripper_calibrations(
    path: Path,
    *,
    robot_id: str,
    calibrations: dict[str, UmiGripperCalibration],
) -> None:
    """Atomically replace the paired UMI endpoint calibration."""
    identity = validate_robot_id(robot_id)
    if set(calibrations) != VALID_SIDES:
        raise ValueError("UMI calibration must contain exactly left and right grippers")
    payload = {
        "schema_version": UMI_GRIPPER_SCHEMA_VERSION,
        "robot_id": identity,
        "grippers": {name: asdict(value) for name, value in sorted(calibrations.items())},
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

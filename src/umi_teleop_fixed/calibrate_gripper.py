"""Supervised, one-sided YAM gripper calibration using i2rt directly."""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import Literal

import tyro

from umi_teleop_fixed.calibration import (
    GripperCalibration,
    calibration_path,
    save_gripper_calibration,
)
from umi_teleop_fixed.can import (
    discover_can_interfaces,
    readiness_error,
    resolve_can_interface,
)

GRIPPER_TYPE = "linear_4310"

LimitDetector = Callable[[str], tuple[float, float]]


@dataclass(frozen=True)
class Args:
    """Calibrate exactly one YAM gripper and cache its raw endpoints."""

    robot_id: str
    """Stable robot identity used to scope the cache file."""

    adapter_serial: str
    """USB serial of the CAN adapter connected to the selected gripper."""

    side: Literal["left", "right"]
    """The only gripper that is allowed to move."""

    def validated_ids(self) -> tuple[str, str]:
        robot_id = self.robot_id.strip()
        serial = self.adapter_serial.strip()
        if not robot_id:
            raise ValueError("robot_id must not be empty")
        if not serial:
            raise ValueError("adapter_serial must not be empty")
        return robot_id, serial


def _detect_i2rt_limits(channel: str) -> tuple[float, float]:
    """Detect endpoints while the YAM arm remains in gravity-compensation idle."""
    try:
        from i2rt.robots.get_robot import get_yam_robot
        from i2rt.robots.utils import ArmType, GripperType
    except ImportError as error:
        raise ImportError("The pinned i2rt dependency is required for calibration") from error

    robot = get_yam_robot(
        channel=channel,
        arm_type=ArmType.YAM,
        gripper_type=GripperType.LINEAR_4310,
        zero_gravity_mode=True,
        gripper_limits_override=None,
        enable_auto_recovery=False,
    )
    try:
        return robot.get_robot_info()["gripper_limits"]
    finally:
        robot.close()


def run(
    args: Args,
    *,
    limit_detector: LimitDetector = _detect_i2rt_limits,
    root: Path | None = None,
    countdown_seconds: int = 3,
    sleep: Callable[[float], None] = time.sleep,
    now: Callable[[], datetime] = lambda: datetime.now(UTC),
) -> Path:
    """Validate, move one gripper through its range, and persist the result."""
    if countdown_seconds < 0:
        raise ValueError("countdown_seconds must be non-negative")

    robot_id, serial = args.validated_ids()
    target_path = calibration_path(robot_id, root=root)
    interfaces = discover_can_interfaces()
    interface = resolve_can_interface(serial, interfaces=interfaces)
    error = readiness_error(interface, bitrate=1_000_000)
    if error is not None:
        raise RuntimeError(f"{args.side} CAN adapter {serial} is not ready: {error}")

    print(
        f"WARNING: {args.side} gripper motor 0x07 will move through its full travel on "
        f"{interface.name}."
    )
    print("Keep hands and objects clear. The arm remains in gravity-compensation idle.")
    for remaining in range(countdown_seconds, 0, -1):
        print(f"Starting in {remaining}...")
        sleep(1.0)

    raw_limits = limit_detector(interface.name)
    limits = GripperCalibration(
        adapter_serial="validation",
        gripper_type=GRIPPER_TYPE,
        gripper_limits=raw_limits,
        calibrated_at_utc="validation",
    ).gripper_limits

    timestamp = now().astimezone(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    calibration = GripperCalibration(
        adapter_serial=serial,
        gripper_type=GRIPPER_TYPE,
        gripper_limits=limits,
        calibrated_at_utc=timestamp,
    )
    save_gripper_calibration(
        target_path,
        robot_id=robot_id,
        side=args.side,
        calibration=calibration,
    )
    print(f"Saved {args.side} gripper limits {limits} to {target_path}")
    return target_path


def main() -> None:
    """CLI entry point."""
    try:
        run(tyro.cli(Args))
    except (ImportError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error


if __name__ == "__main__":
    main()

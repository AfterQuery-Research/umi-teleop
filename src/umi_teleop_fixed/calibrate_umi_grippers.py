from __future__ import annotations

import math
import statistics
import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

import tyro

from .calibration import (
    UmiGripperCalibration,
    save_umi_gripper_calibrations,
    umi_gripper_calibration_path,
    validate_robot_id,
)
from .gripper_receiver import GripperReceiver
from .robot import SIDES


@dataclass(frozen=True)
class Args:
    """Calibrate raw closed/open widths for both camera-tracked UMI grippers."""

    robot_id: str
    sender_ip: str
    port: int = 9130
    bind_host: str = "0.0.0.0"
    gripper_timeout_seconds: float = 0.5
    stable_seconds: float = 1.0
    stable_range_mm: float = 0.8
    minimum_span_mm: float = 20.0
    instruction_delay_seconds: float = 1.0
    stage_timeout_seconds: float = 60.0

    def validate(self) -> None:
        validate_robot_id(self.robot_id)
        positive = {
            "gripper_timeout_seconds": self.gripper_timeout_seconds,
            "stable_seconds": self.stable_seconds,
            "stable_range_mm": self.stable_range_mm,
            "minimum_span_mm": self.minimum_span_mm,
            "instruction_delay_seconds": self.instruction_delay_seconds,
            "stage_timeout_seconds": self.stage_timeout_seconds,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive")
        if self.stage_timeout_seconds <= self.stable_seconds + self.instruction_delay_seconds:
            raise ValueError("stage_timeout_seconds is too short")


def _capture_endpoint(
    receiver: GripperReceiver,
    *,
    args: Args,
    side: str,
    instruction: str,
    accept: Callable[[float], bool],
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
    emit: Callable[[str], None],
) -> float:
    emit(instruction)
    emit("Move only as instructed, then hold still; capture advances automatically.")
    started = monotonic()
    collect_after = started + args.instruction_delay_seconds
    deadline = started + args.stage_timeout_seconds
    next_status = started
    buffer: list[tuple[float, float]] = []
    while monotonic() < deadline:
        now = monotonic()
        samples, errors = receiver.read(
            now=now,
            stale_after_seconds=args.gripper_timeout_seconds,
        )
        sample = samples.get(side)
        if now < collect_after or sample is None or not accept(sample.width_mm):
            buffer.clear()
        else:
            candidate_widths = [width for _, width in buffer]
            candidate_widths.append(sample.width_mm)
            if max(candidate_widths) - min(candidate_widths) > args.stable_range_mm:
                buffer = [(now, sample.width_mm)]
            else:
                buffer.append((now, sample.width_mm))
        if buffer and now - buffer[0][0] >= args.stable_seconds:
            widths = [width for _, width in buffer]
            value = float(statistics.median(widths))
            emit(f"Captured {side}: {value:.2f} mm.")
            return value
        if now >= next_status:
            if sample is None:
                status = f"waiting({errors.get(side, 'unavailable')})"
            elif not accept(sample.width_mm):
                status = (
                    f"{sample.width_mm:.2f}mm "
                    "(waiting for the requested movement)"
                )
            elif buffer:
                held_seconds = max(0.0, now - buffer[0][0])
                status = (
                    f"{sample.width_mm:.2f}mm "
                    f"(stable {held_seconds:.1f}/{args.stable_seconds:g}s)"
                )
            else:
                status = f"{sample.width_mm:.2f}mm"
            emit(f"Waiting for stable {side} endpoint: {status}")
            next_status = now + 1.0
        sleep(0.02)
    raise RuntimeError(f"timed out after {args.stage_timeout_seconds:g}s: {instruction}")


def _capture_starting_open_widths(
    receiver: GripperReceiver,
    *,
    args: Args,
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
    emit: Callable[[str], None],
) -> dict[str, float]:
    emit("Starting condition: fully OPEN both UMI grippers and hold them still.")
    emit("The measured widths are discovered from the camera stream; no endpoint is assumed.")
    started = monotonic()
    collect_after = started + args.instruction_delay_seconds
    deadline = started + args.stage_timeout_seconds
    next_status = started
    buffer: list[tuple[float, dict[str, float]]] = []
    while monotonic() < deadline:
        now = monotonic()
        samples, errors = receiver.read(
            now=now,
            stale_after_seconds=args.gripper_timeout_seconds,
        )
        if now < collect_after or any(side not in samples for side in SIDES):
            buffer.clear()
        else:
            widths = {side: samples[side].width_mm for side in SIDES}
            candidate = [entry_widths for _, entry_widths in buffer]
            candidate.append(widths)
            stable = all(
                max(entry[side] for entry in candidate)
                - min(entry[side] for entry in candidate)
                <= args.stable_range_mm
                for side in SIDES
            )
            if stable:
                buffer.append((now, widths))
            else:
                buffer = [(now, widths)]

        if buffer and now - buffer[0][0] >= args.stable_seconds:
            captured = {
                side: float(statistics.median(widths[side] for _, widths in buffer))
                for side in SIDES
            }
            emit(
                f"Captured starting OPEN widths: left={captured['left']:.2f} mm, "
                f"right={captured['right']:.2f} mm."
            )
            return captured

        if now >= next_status:
            status = ", ".join(
                f"{side}="
                + (
                    f"{samples[side].width_mm:.2f}mm"
                    if side in samples
                    else f"waiting({errors.get(side, 'unavailable')})"
                )
                for side in SIDES
            )
            emit(f"Waiting for stable starting OPEN widths: {status}")
            next_status = now + 1.0
        sleep(0.02)
    raise RuntimeError("timed out waiting for stable open left and right gripper widths")


ReceiverFactory = Callable[[Args], GripperReceiver]


def _receiver(args: Args) -> GripperReceiver:
    return GripperReceiver(
        sender_ip=args.sender_ip,
        port=args.port,
        bind_host=args.bind_host,
    )


def run(
    args: Args,
    *,
    receiver_factory: ReceiverFactory = _receiver,
    root: Path | None = None,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    emit: Callable[[str], None] = print,
    now: Callable[[], datetime] = lambda: datetime.now(UTC),
) -> Path:
    args.validate()
    with receiver_factory(args) as receiver:
        starting_open_widths = _capture_starting_open_widths(
            receiver,
            args=args,
            monotonic=monotonic,
            sleep=sleep,
            emit=emit,
        )
        close_widths = {}
        open_widths = {}
        close_widths["left"] = _capture_endpoint(
            receiver,
            args=args,
            side="left",
            instruction="Fully CLOSE the LEFT UMI gripper and hold it still.",
            accept=lambda width: starting_open_widths["left"] - width
            >= args.minimum_span_mm,
            monotonic=monotonic,
            sleep=sleep,
            emit=emit,
        )
        open_widths["left"] = _capture_endpoint(
            receiver,
            args=args,
            side="left",
            instruction="Fully OPEN the LEFT UMI gripper and hold it still.",
            accept=lambda width: width - close_widths["left"] >= args.minimum_span_mm,
            monotonic=monotonic,
            sleep=sleep,
            emit=emit,
        )
        close_widths["right"] = _capture_endpoint(
            receiver,
            args=args,
            side="right",
            instruction="Fully CLOSE the RIGHT UMI gripper and hold it still.",
            accept=lambda width: starting_open_widths["right"] - width
            >= args.minimum_span_mm,
            monotonic=monotonic,
            sleep=sleep,
            emit=emit,
        )
        open_widths["right"] = _capture_endpoint(
            receiver,
            args=args,
            side="right",
            instruction="Fully OPEN the RIGHT UMI gripper and hold it still.",
            accept=lambda width: width - close_widths["right"] >= args.minimum_span_mm,
            monotonic=monotonic,
            sleep=sleep,
            emit=emit,
        )

    timestamp = now().astimezone(UTC).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    calibrations = {
        side: UmiGripperCalibration(
            sender_ip=args.sender_ip,
            closed_width_mm=close_widths[side],
            open_width_mm=open_widths[side],
            calibrated_at_utc=timestamp,
        )
        for side in SIDES
    }
    too_small = [
        side for side, value in calibrations.items() if value.span_mm < args.minimum_span_mm
    ]
    if too_small:
        raise RuntimeError(
            f"UMI width span is too small for {', '.join(too_small)}; calibration was not saved"
        )
    path = umi_gripper_calibration_path(args.robot_id, root=root)
    save_umi_gripper_calibrations(
        path,
        robot_id=args.robot_id,
        calibrations=calibrations,
    )
    emit(
        "Saved UMI spans: "
        + ", ".join(f"{side}={calibrations[side].span_mm:.2f} mm" for side in SIDES)
        + f" to {path}"
    )
    return path


def main() -> None:
    try:
        run(tyro.cli(Args))
    except KeyboardInterrupt as error:
        raise SystemExit(130) from error
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error


if __name__ == "__main__":
    main()

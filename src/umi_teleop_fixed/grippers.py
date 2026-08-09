from __future__ import annotations

import time
from dataclasses import dataclass

import tyro

from .gripper_receiver import GripperReceiver


@dataclass(frozen=True)
class Args:
    """Inspect the raw UMI gripper-width stream without connecting a robot."""

    sender_ip: str
    port: int = 9130
    bind_host: str = "0.0.0.0"
    timeout_seconds: float = 0.2
    duration_seconds: float | None = None

    def validate(self) -> None:
        if self.timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be positive")
        if self.duration_seconds is not None and self.duration_seconds <= 0:
            raise ValueError("duration_seconds must be positive when provided")


def run(args: Args) -> None:
    args.validate()
    started = time.monotonic()
    next_status = started
    with GripperReceiver(
        sender_ip=args.sender_ip,
        port=args.port,
        bind_host=args.bind_host,
    ) as receiver:
        print(f"Listening for UMI gripper widths from {args.sender_ip} on UDP {args.port}.")
        while args.duration_seconds is None or time.monotonic() - started < args.duration_seconds:
            now = time.monotonic()
            if now >= next_status:
                try:
                    samples = receiver.snapshot(now=now, stale_after_seconds=args.timeout_seconds)
                    left = samples["left"]
                    right = samples["right"]
                    stats = receiver.stats()
                    left_age = now - left.received_at + left.camera_age_seconds
                    right_age = now - right.received_at + right.camera_age_seconds
                    print(
                        f"left={left.width_mm:.2f}mm age={left_age:.3f}s "
                        f"right={right.width_mm:.2f}mm "
                        f"age={right_age:.3f}s "
                        f"packets={stats.packets} malformed={stats.malformed} "
                        f"out_of_order={stats.out_of_order}",
                        flush=True,
                    )
                except RuntimeError as error:
                    print(f"WAITING: {error}", flush=True)
                next_status = now + 1.0
            time.sleep(0.02)


def main() -> None:
    try:
        run(tyro.cli(Args))
    except KeyboardInterrupt as error:
        raise SystemExit(130) from error
    except (OSError, RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error


if __name__ == "__main__":
    main()

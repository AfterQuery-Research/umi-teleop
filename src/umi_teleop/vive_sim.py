"""Deterministic dual-tracker sender used only by non-moving integration tests."""

from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass

import tyro


@dataclass(frozen=True)
class Args:
    target_host: str = "127.0.0.1"
    target_port: int = 9001
    rate_hz: float = 120.0
    left_id: str = "LHR-BBAF093A"
    right_id: str = "LHR-C891E882"
    max_packets: int | None = None


def run(args: Args) -> None:
    if not 1 <= args.target_port <= 65_535 or not 1.0 <= args.rate_hz <= 240.0:
        raise ValueError("target port or rate is out of range")
    target = (socket.gethostbyname(args.target_host), args.target_port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    period = 1.0 / args.rate_hz
    next_send = time.monotonic()
    seq = 0
    try:
        while args.max_packets is None or seq < args.max_packets:
            packet = {
                "frame": "libsurvive_tracking",
                "trackers": [
                    {
                        "serial": args.left_id,
                        "valid": True,
                        "pose_age_s": 0.0,
                        "pos": [0.0, 0.0, 0.0],
                        "quat": [1.0, 0.0, 0.0, 0.0],
                    },
                    {
                        "serial": args.right_id,
                        "valid": True,
                        "pose_age_s": 0.0,
                        "pos": [0.2, 0.0, 0.0],
                        "quat": [1.0, 0.0, 0.0, 0.0],
                    },
                ],
            }
            sock.sendto(
                json.dumps(packet, allow_nan=False, separators=(",", ":")).encode(),
                target,
            )
            seq += 1
            next_send += period
            time.sleep(max(0.0, next_send - time.monotonic()))
    finally:
        sock.close()


def main() -> None:
    try:
        run(tyro.cli(Args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

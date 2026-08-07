#!/usr/bin/env python3
"""Measure the native Vive UDP stream without importing umi-teleop or touching CAN."""

from __future__ import annotations

import argparse
import json
import socket
import statistics
import time
from collections import defaultdict


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9001)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--expect-serial", action="append", default=[])
    args = parser.parse_args()
    if not 0 < args.port < 65536 or args.duration_s <= 0:
        parser.error("port and duration must be positive")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(min(1.0, args.duration_s))
    started = time.monotonic()
    deadline = started + args.duration_s
    arrivals: list[float] = []
    valid = defaultdict(int)
    observed = defaultdict(int)
    pose_ages_ms = defaultdict(list)
    bad_packets = 0
    frame_names: set[str] = set()
    try:
        while time.monotonic() < deadline:
            try:
                payload, _address = sock.recvfrom(65_535)
            except TimeoutError:
                continue
            arrived = time.monotonic()
            try:
                packet = json.loads(payload)
                trackers = packet["trackers"]
                if not isinstance(trackers, list):
                    raise ValueError("trackers is not a list")
            except (KeyError, TypeError, ValueError, json.JSONDecodeError, UnicodeDecodeError):
                bad_packets += 1
                continue
            arrivals.append(arrived)
            frame_names.add(str(packet.get("frame", "openvr")))
            for tracker in trackers:
                serial = str(tracker.get("serial", ""))
                if not serial:
                    continue
                observed[serial] += 1
                if tracker.get("valid"):
                    valid[serial] += 1
                    pose_ages_ms[serial].append(float(tracker["pose_age_s"]) * 1000.0)
    finally:
        sock.close()

    duration = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0.0
    gaps_ms = [1000.0 * (right - left) for left, right in zip(arrivals, arrivals[1:], strict=False)]
    serials = sorted(observed)
    summary = {
        "packets": len(arrivals),
        "bad_packets": bad_packets,
        "duration_s": duration,
        "effective_hz": (len(arrivals) - 1) / duration if duration > 0 else 0.0,
        "gap_p50_ms": statistics.median(gaps_ms) if gaps_ms else None,
        "gap_p95_ms": percentile(gaps_ms, 0.95),
        "frames": sorted(frame_names),
        "trackers": {
            serial: {
                "valid_percent": 100.0 * valid[serial] / observed[serial],
                "pose_age_p95_ms": percentile(pose_ages_ms[serial], 0.95),
            }
            for serial in serials
        },
    }
    print(json.dumps(summary, indent=2, sort_keys=True))

    missing = sorted(set(args.expect_serial) - set(serials))
    if not arrivals or bad_packets or missing:
        if missing:
            print(f"missing expected tracker serials: {', '.join(missing)}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

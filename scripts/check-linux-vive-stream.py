#!/usr/bin/env python3
"""Measure the native Vive UDP stream without importing umi-teleop or touching CAN."""

from __future__ import annotations

import argparse
import json
import math
import socket
import statistics
import time
from collections import defaultdict


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))]


def quaternion_angle_rad(left: list[float], right: list[float]) -> float:
    left_norm = math.sqrt(sum(value * value for value in left))
    right_norm = math.sqrt(sum(value * value for value in right))
    dot = sum(a * b for a, b in zip(left, right, strict=True)) / (left_norm * right_norm)
    return 2.0 * math.acos(min(1.0, max(0.0, abs(dot))))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9001)
    parser.add_argument("--duration-s", type=float, default=10.0)
    parser.add_argument("--expect-serial", action="append", default=[])
    parser.add_argument("--min-valid-percent", type=float, default=95.0)
    parser.add_argument("--max-linear-speed-m-s", type=float, default=8.0)
    parser.add_argument("--max-angular-speed-deg-s", type=float, default=1440.0)
    args = parser.parse_args()
    if (
        not 0 < args.port < 65536
        or args.duration_s <= 0
        or not 0.0 <= args.min_valid_percent <= 100.0
        or args.max_linear_speed_m_s <= 0.0
        or args.max_angular_speed_deg_s <= 0.0
    ):
        parser.error("port, duration, validity, and speed limits must be positive and valid")

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
    last_continuity: dict[str, int] = {}
    last_rejected: dict[str, int] = {}
    continuity_resets = defaultdict(int)
    new_rejections = defaultdict(int)
    timecode_regressions = defaultdict(int)
    rate_violations = defaultdict(int)
    linear_speeds = defaultdict(list)
    angular_speeds_deg_s = defaultdict(list)
    last_samples: dict[str, tuple[int, int, list[float], list[float]]] = {}
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
                try:
                    continuity_id = int(tracker.get("continuity_id", 0))
                    rejected_poses = int(tracker.get("rejected_poses", 0))
                    if serial in last_continuity and continuity_id != last_continuity[serial]:
                        continuity_resets[serial] += 1
                    if serial in last_rejected and rejected_poses > last_rejected[serial]:
                        new_rejections[serial] += rejected_poses - last_rejected[serial]
                    last_continuity[serial] = continuity_id
                    last_rejected[serial] = rejected_poses
                except (TypeError, ValueError):
                    bad_packets += 1
                    continue
                if tracker.get("valid"):
                    try:
                        pose = [float(value) for value in tracker["pos"]]
                        quaternion = [float(value) for value in tracker["quat"]]
                        timecode = int(tracker["device_timecode"])
                        timebase_hz = int(tracker["timebase_hz"])
                        if (
                            len(pose) != 3
                            or len(quaternion) != 4
                            or not all(math.isfinite(value) for value in pose + quaternion)
                            or timecode <= 0
                            or timebase_hz <= 0
                        ):
                            raise ValueError("invalid tracker sample")
                        valid[serial] += 1
                        pose_ages_ms[serial].append(float(tracker["pose_age_s"]) * 1000.0)
                    except (KeyError, TypeError, ValueError):
                        bad_packets += 1
                        continue
                    previous = last_samples.get(serial)
                    if previous is not None and previous[0] == continuity_id:
                        previous_timecode, previous_pose, previous_quaternion = (
                            previous[1],
                            previous[2],
                            previous[3],
                        )
                        if timecode < previous_timecode:
                            timecode_regressions[serial] += 1
                            continue
                        if timecode > previous_timecode:
                            delta_s = (timecode - previous_timecode) / timebase_hz
                            linear_speed = (
                                math.sqrt(
                                    sum(
                                        (a - b) ** 2
                                        for a, b in zip(pose, previous_pose, strict=True)
                                    )
                                )
                                / delta_s
                            )
                            angular_speed = math.degrees(
                                quaternion_angle_rad(previous_quaternion, quaternion) / delta_s
                            )
                            linear_speeds[serial].append(linear_speed)
                            angular_speeds_deg_s[serial].append(angular_speed)
                            if (
                                linear_speed > args.max_linear_speed_m_s
                                or angular_speed > args.max_angular_speed_deg_s
                            ):
                                rate_violations[serial] += 1
                    last_samples[serial] = (continuity_id, timecode, pose, quaternion)
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
                "continuity_resets": continuity_resets[serial],
                "new_rejected_poses": new_rejections[serial],
                "timecode_regressions": timecode_regressions[serial],
                "rate_violations": rate_violations[serial],
                "max_linear_speed_m_s": max(linear_speeds[serial], default=None),
                "max_angular_speed_deg_s": max(angular_speeds_deg_s[serial], default=None),
            }
            for serial in serials
        },
    }
    print(json.dumps(summary, indent=2, sort_keys=True))

    missing = sorted(set(args.expect_serial) - set(serials))
    unhealthy = [
        serial
        for serial in serials
        if 100.0 * valid[serial] / observed[serial] < args.min_valid_percent
        or continuity_resets[serial]
        or new_rejections[serial]
        or timecode_regressions[serial]
        or rate_violations[serial]
    ]
    if not arrivals or bad_packets or missing or unhealthy:
        if missing:
            print(f"missing expected tracker serials: {', '.join(missing)}")
        if unhealthy:
            print(f"unhealthy tracker streams: {', '.join(unhealthy)}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

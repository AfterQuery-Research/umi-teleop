"""Stream all valid SteamVR trackers from Windows to the UMI source Pi."""

from __future__ import annotations

import json
import math
import socket
import time
from dataclasses import dataclass
from typing import Any

import tyro


@dataclass(frozen=True)
class Config:
    """Windows SteamVR sender settings."""

    target_host: str = "10.103.3.43"
    target_port: int = 9001
    rate_hz: float = 50.0


def matrix_to_xyz(matrix: Any) -> tuple[float, float, float]:
    return float(matrix[0][3]), float(matrix[1][3]), float(matrix[2][3])


def matrix_to_quaternion(matrix: Any) -> tuple[float, float, float, float]:
    """Convert an OpenVR 3x4 transform to a normalized quaternion in wxyz order."""
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / scale
        x = (matrix[2][1] - matrix[1][2]) * scale
        y = (matrix[0][2] - matrix[2][0]) * scale
        z = (matrix[1][0] - matrix[0][1]) * scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = 2.0 * math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2])
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = 2.0 * math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2])
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = 2.0 * math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1])
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    return w / norm, x / norm, y / norm, z / norm


def tracker_packet(
    vr_system: Any,
    openvr: Any,
    *,
    timestamp: float | None = None,
) -> dict[str, Any]:
    """Read one SteamVR frame and return the wire-format tracker packet."""
    poses = vr_system.getDeviceToAbsoluteTrackingPose(
        openvr.TrackingUniverseStanding,
        0,
        openvr.k_unMaxTrackedDeviceCount,
    )
    trackers = []
    for index in range(openvr.k_unMaxTrackedDeviceCount):
        if vr_system.getTrackedDeviceClass(index) != openvr.TrackedDeviceClass_GenericTracker:
            continue
        serial = vr_system.getStringTrackedDeviceProperty(index, openvr.Prop_SerialNumber_String)
        pose = poses[index]
        if not pose.bPoseIsValid:
            trackers.append({"serial": serial, "valid": False})
            continue
        matrix = pose.mDeviceToAbsoluteTracking
        trackers.append(
            {
                "serial": serial,
                "valid": True,
                "pos": list(matrix_to_xyz(matrix)),
                "quat": list(matrix_to_quaternion(matrix)),
            }
        )
    return {"t": time.time() if timestamp is None else timestamp, "trackers": trackers}


def run(config: Config) -> None:
    if config.rate_hz <= 0:
        raise ValueError("rate-hz must be positive")
    try:
        import openvr
    except ImportError as exc:  # pragma: no cover - depends on the Windows extra
        raise SystemExit("OpenVR is missing; run `uv sync --extra windows` on Windows") from exc

    openvr.init(openvr.VRApplication_Background)
    vr_system = openvr.VRSystem()
    target = (config.target_host, config.target_port)
    period = 1.0 / config.rate_hz
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(
        f"Streaming SteamVR trackers to {config.target_host}:{config.target_port} "
        f"at {config.rate_hz:g} Hz (Ctrl+C to stop)",
        flush=True,
    )
    try:
        next_send = time.monotonic()
        while True:
            packet = tracker_packet(vr_system, openvr)
            sock.sendto(json.dumps(packet, allow_nan=False, separators=(",", ":")).encode(), target)
            next_send += period
            time.sleep(max(0.0, next_send - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        openvr.shutdown()


def main() -> None:
    run(tyro.cli(Config))


if __name__ == "__main__":
    main()

"""Live LiDAR/GLIM pose plumbing for UMI teleoperation.

The live GLIM pipeline publishes odometry in GLIM's ROS frame.  The teleop controller expects the
same UMI-handle convention used by the Vive path: +X up, +Y right, +Z forward before the YAM-side
``umi-handle`` zero-frame correction.  This module keeps the frame conversion and small UDP
protocols testable and independent of Docker/ROS.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from contextlib import suppress
from dataclasses import dataclass
from typing import Any

import numpy as np

from umi_teleop.protocol import ARM_NAMES, MAX_DATAGRAM_BYTES, RECEIVE_BUFFER_BYTES
from umi_teleop.vive import matrix_to_quat_xyzw

LIDAR_PROTOCOL_VERSION = 1

# From umi-flow/mac/live_pose_plot.py.  p_umi = GLIM_TO_UMI @ p_glim, with UMI axes
# ordered as (up, right, forward).  This is intentionally a proper rotation.
GLIM_TO_UMI = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)

# RoboSense lidar body -> UMI TCP lever from umi-flow/mac/live_pose_plot.py.  This is not
# applied to outgoing poses by default; the YAM controller applies tip offsets so zeroing and
# rotation limits stay identical to the Vive path.
DEFAULT_LIDAR_TIP_OFFSET_M = (-0.10479, 0.0, 0.22244)


def quat_xyzw_to_matrix(quat_xyzw: Any) -> np.ndarray:
    q = np.asarray(quat_xyzw, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(q))
    if norm < 1e-9:
        raise ValueError("quaternion has zero norm")
    x, y, z, w = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def glim_odom_to_umi_pose(
    position_xyz: Any,
    orientation_xyzw: Any,
    *,
    tip_offset_m: tuple[float, float, float] | None = None,
) -> list[float]:
    """Convert GLIM odometry to a teleop pose in UMI-handle coordinates.

    By default the returned pose is the LiDAR body pose.  If ``tip_offset_m`` is supplied,
    translation is shifted to the TCP while orientation remains the LiDAR orientation.  The live
    teleop path should normally leave this unset and pass the LiDAR tip offset to ``umi-yam``.
    """

    p_glim = np.asarray(position_xyz, dtype=np.float64)
    if p_glim.shape != (3,) or not np.isfinite(p_glim).all():
        raise ValueError("position must contain three finite values")
    r_glim = quat_xyzw_to_matrix(orientation_xyzw)
    r_umi = GLIM_TO_UMI @ r_glim @ GLIM_TO_UMI.T
    p_umi = GLIM_TO_UMI @ p_glim
    if tip_offset_m is not None:
        offset = np.asarray(tip_offset_m, dtype=np.float64)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("tip_offset_m must contain three finite values")
        p_umi = p_umi + r_umi @ offset
    quat = matrix_to_quat_xyzw(r_umi)
    return [float(v) for v in (*p_umi, *quat)]


def _finite(value: Any, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{name} must be finite")
    if minimum is not None and out < minimum:
        raise ValueError(f"{name} must be >= {minimum}")
    return out


def _counter(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _validate_pose_arm(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"arms.{name} must be an object")
    pose = value.get("pose")
    if not isinstance(pose, list) or len(pose) != 7:
        raise ValueError(f"arms.{name}.pose must contain xyz + quaternion xyzw")
    pose = [_finite(v, f"arms.{name}.pose") for v in pose]
    qnorm = math.sqrt(sum(v * v for v in pose[3:]))
    if qnorm < 1e-6 or abs(qnorm - 1.0) > 1e-3:
        raise ValueError(f"arms.{name}.pose quaternion must be normalized")
    return {
        "serial": str(value.get("serial", "")),
        "pose": pose,
        "source_time_s": _finite(value.get("source_time_s"), f"arms.{name}.source_time_s"),
    }


def decode_lidar_pose_packet(data: bytes) -> dict[str, Any]:
    """Decode Lambda-to-merger LiDAR pose datagrams."""

    if not data or len(data) > MAX_DATAGRAM_BYTES:
        raise ValueError("LiDAR pose datagram is empty or too large")
    packet = json.loads(data.decode("utf-8"))
    if not isinstance(packet, dict):
        raise ValueError("LiDAR pose packet must be a JSON object")
    if packet.get("version") != LIDAR_PROTOCOL_VERSION:
        raise ValueError(f"unsupported LiDAR protocol version {packet.get('version')!r}")
    arms = packet.get("arms")
    if not isinstance(arms, dict):
        raise ValueError("arms must be an object")
    out: dict[str, dict[str, Any]] = {}
    for name in ARM_NAMES:
        if name in arms:
            out[name] = _validate_pose_arm(arms[name], name)
    if not out:
        raise ValueError("LiDAR pose packet must contain at least one arm")
    return {
        "version": LIDAR_PROTOCOL_VERSION,
        "source": str(packet.get("source", "lidar-glim")),
        "seq": _counter(packet.get("seq"), "seq"),
        "arms": out,
    }


def encode_lidar_pose_packet(packet: dict[str, Any]) -> bytes:
    clean = decode_lidar_pose_packet(
        json.dumps(packet, allow_nan=False, separators=(",", ":")).encode("utf-8")
    )
    return json.dumps(clean, allow_nan=False, separators=(",", ":")).encode("utf-8")


def decode_lidar_arm_packet(data: bytes) -> dict[str, Any]:
    """Decode UMI-Pi-to-merger width/control datagrams."""

    if not data or len(data) > MAX_DATAGRAM_BYTES:
        raise ValueError("LiDAR arm datagram is empty or too large")
    packet = json.loads(data.decode("utf-8"))
    if not isinstance(packet, dict):
        raise ValueError("LiDAR arm packet must be a JSON object")
    if packet.get("version") != LIDAR_PROTOCOL_VERSION:
        raise ValueError(f"unsupported LiDAR protocol version {packet.get('version')!r}")
    arm = str(packet.get("arm", ""))
    if arm not in ARM_NAMES:
        raise ValueError("arm must be 'left' or 'right'")
    width = packet.get("width_mm")
    width_age = packet.get("width_age_s")
    if width is None:
        if width_age is not None:
            raise ValueError("width_age_s must be null when width_mm is null")
    else:
        width = _finite(width, "width_mm")
        width_age = _finite(width_age, "width_age_s", minimum=0.0)
    if not isinstance(packet.get("requested_enabled"), bool):
        raise ValueError("requested_enabled must be boolean")
    return {
        "version": LIDAR_PROTOCOL_VERSION,
        "session_id": str(packet.get("session_id", "")),
        "source_id": str(packet.get("source_id", arm)),
        "arm": arm,
        "seq": _counter(packet.get("seq"), "seq"),
        "requested_enabled": packet["requested_enabled"],
        "zero_id": _counter(packet.get("zero_id"), "zero_id"),
        "toggle_id": _counter(packet.get("toggle_id"), "toggle_id"),
        "cancel_id": _counter(packet.get("cancel_id"), "cancel_id"),
        "quit_id": _counter(packet.get("quit_id"), "quit_id"),
        "width_mm": width,
        "width_age_s": width_age,
    }


def encode_lidar_arm_packet(packet: dict[str, Any]) -> bytes:
    clean = decode_lidar_arm_packet(
        json.dumps(packet, allow_nan=False, separators=(",", ":")).encode("utf-8")
    )
    return json.dumps(clean, allow_nan=False, separators=(",", ":")).encode("utf-8")


@dataclass(frozen=True)
class _Stamped:
    payload: dict[str, Any]
    received_at: float


class LidarPoseReceiver:
    """Receive live GLIM poses and expose them in the normal teleop pose snapshot shape."""

    def __init__(self, host: str = "0.0.0.0", port: int = 9142):
        self.host = host
        self.port = port
        self.packet_count = 0
        self.bad_packet_count = 0
        self._latest: dict[str, _Stamped] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        with suppress(OSError):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER_BYTES)
        sock.bind((self.host, self.port))
        self.port = sock.getsockname()[1]
        sock.settimeout(0.5)
        self._sock = sock
        self._stop.clear()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65_535)  # type: ignore[union-attr]
            except TimeoutError:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                continue
            try:
                packet = decode_lidar_pose_packet(data)
            except (json.JSONDecodeError, UnicodeDecodeError, TypeError, ValueError):
                self.bad_packet_count += 1
                continue
            now = time.monotonic()
            with self._lock:
                self.packet_count += 1
                for arm, payload in packet["arms"].items():
                    self._latest[arm] = _Stamped(payload, now)

    def snapshot(self, now: float | None = None) -> dict[str, dict[str, Any]]:
        now = time.monotonic() if now is None else now
        with self._lock:
            latest = dict(self._latest)
        return {
            arm: {
                "serial": stamped.payload["serial"],
                "pose": stamped.payload["pose"],
                "age_s": max(0.0, now - stamped.received_at),
            }
            for arm, stamped in latest.items()
        }

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            self._sock.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self._sock = None
        with self._lock:
            self._latest.clear()


class LidarArmReceiver:
    """Receive per-UMI width/control datagrams."""

    def __init__(self, host: str = "0.0.0.0", port: int = 9143):
        self.host = host
        self.port = port
        self.packet_count = 0
        self.bad_packet_count = 0
        self._latest: dict[str, _Stamped] = {}
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._sock: socket.socket | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        with suppress(OSError):
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER_BYTES)
        sock.bind((self.host, self.port))
        self.port = sock.getsockname()[1]
        sock.settimeout(0.5)
        self._sock = sock
        self._stop.clear()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def _recv_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65_535)  # type: ignore[union-attr]
            except TimeoutError:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                continue
            try:
                packet = decode_lidar_arm_packet(data)
            except (json.JSONDecodeError, UnicodeDecodeError, TypeError, ValueError):
                self.bad_packet_count += 1
                continue
            with self._lock:
                current = self._latest.get(packet["source_id"])
                if (
                    current is None
                    or packet["session_id"] != current.payload["session_id"]
                    or packet["seq"] > current.payload["seq"]
                ):
                    self.packet_count += 1
                    self._latest[packet["source_id"]] = _Stamped(packet, time.monotonic())

    def latest_packets(self) -> dict[str, dict[str, Any]]:
        with self._lock:
            return {key: stamped.payload.copy() for key, stamped in self._latest.items()}

    def widths(self, now: float | None = None) -> dict[str, dict[str, Any]]:
        now = time.monotonic() if now is None else now
        with self._lock:
            latest = list(self._latest.values())
        out = {name: {"width_mm": None, "age_s": None} for name in ARM_NAMES}
        for stamped in latest:
            arm = stamped.payload["arm"]
            width = stamped.payload["width_mm"]
            if width is None:
                continue
            out[arm] = {
                "width_mm": width,
                "age_s": max(stamped.payload["width_age_s"], now - stamped.received_at),
            }
        return out

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            self._sock.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self._sock = None
        with self._lock:
            self._latest.clear()

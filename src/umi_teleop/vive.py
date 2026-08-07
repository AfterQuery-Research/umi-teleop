"""Pure-numpy Vive helpers shared by the source receiver and multi-UMI monitor.

The Windows OpenVR and native Linux libsurvive senders emit one UDP packet holding all trackers
(``{"t": epoch, "trackers": [{"serial", "valid", "pos", "quat"}, ...]}``) in one global
lighthouse frame. This module owns:

- the tracking-frame to ROS conversion used by the source and monitor, and
- :func:`parse_trackers` / :class:`MultiViveZeroer` for the multi-tracker Mac monitor.

Everything here is numpy-only (no ROS, no sockets) so it runs unchanged on every host and is
unit-testable. Quaternions are ``[x, y, z, w]`` unless a name says ``wxyz``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from umi_teleop.pose_zero import _conjugate, _multiply, _normalize, _rotate

# OpenVR: +X right, +Y up, -Z forward.  ROS REP-103: +X forward, +Y left, +Z up.
OPENVR_TO_ROS = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)

# Tracker body-axis correction (180 deg about X), right-multiplied on the BODY side so the
# body-relative poses downstream read forward +Z / right +Y / up +X. See run_vive_receiver.
R_BODY = np.diag([1.0, -1.0, -1.0])

# (x, y, z, roll, pitch, yaw): position in metres, orientation in degrees.
Pose6 = tuple[float, float, float, float, float, float]
# ROS-frame tracker pose: (position[3], quaternion_xyzw[4]).
TrackerPose = tuple[np.ndarray, np.ndarray]


@dataclass(frozen=True)
class VivePose:
    serial: str
    position: np.ndarray
    orientation_xyzw: np.ndarray


@dataclass(frozen=True)
class TrackerUpdate:
    """One wire update plus its age on the tracker host."""

    pose: TrackerPose | None
    sample_age_s: float = 0.0


def normalize(q: np.ndarray) -> np.ndarray:
    """L2-normalize a quaternion, rejecting non-finite / zero-norm input (strict)."""
    if not np.isfinite(q).all():
        raise ValueError("quaternion must contain only finite values")
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm == 0.0:
        raise ValueError("quaternion has zero or non-finite norm")
    return q / norm


def quat_wxyz_to_matrix(q: np.ndarray) -> np.ndarray:
    w, x, y, z = normalize(q)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def matrix_to_quat_xyzw(m: np.ndarray) -> np.ndarray:
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return normalize(np.array([x, y, z, w], dtype=np.float64))


def openvr_pose_to_ros(pos: Any, quat_wxyz: Any) -> TrackerPose:
    """Convert an OpenVR (pos, quat_wxyz) to a ROS-frame (pos, quat_xyzw) pose."""
    p_vr = np.asarray(pos, dtype=np.float64)
    q_vr = np.asarray(quat_wxyz, dtype=np.float64)
    if p_vr.shape != (3,):
        raise ValueError(f"pos must have 3 values, got {p_vr.shape}")
    if q_vr.shape != (4,):
        raise ValueError(f"quat must have 4 values, got {q_vr.shape}")
    if not np.isfinite(p_vr).all():
        raise ValueError("pos must contain only finite values")
    if not np.isfinite(q_vr).all():
        raise ValueError("quat must contain only finite values")

    p_ros = OPENVR_TO_ROS @ p_vr
    r_vr = quat_wxyz_to_matrix(q_vr)
    r_ros = OPENVR_TO_ROS @ r_vr @ OPENVR_TO_ROS.T @ R_BODY
    return p_ros, matrix_to_quat_xyzw(r_ros)


def libsurvive_pose_to_ros(pos: Any, quat_wxyz: Any) -> TrackerPose:
    """Convert libsurvive's object-to-tracking pose into the established control basis.

    Both backends expose object-to-world transforms with ``wxyz`` quaternions. Libsurvive's
    calibrated world orientation is arbitrary, but the normal YAM setup captures the right UMI's
    orientation at ZERO and therefore removes that constant world basis. Applying the established
    basis and tracker body correction here preserves the downstream packet contract and handle-axis
    behavior while keeping Windows packets backward compatible.
    """
    return openvr_pose_to_ros(pos, quat_wxyz)


def _trackers(packet: Any) -> list[Any]:
    if not isinstance(packet, dict):
        raise ValueError("packet must be a JSON object")
    trackers = packet.get("trackers", [])
    if not isinstance(trackers, list):
        raise ValueError("packet['trackers'] must be a list")
    return trackers


def _packet_frame(packet: dict[str, Any]) -> str:
    frame = packet.get("frame", "openvr")
    if frame not in ("openvr", "libsurvive_tracking"):
        raise ValueError(f"unsupported Vive packet frame {frame!r}")
    return str(frame)


def _tracker_pose_fields(tracker: Any) -> tuple[str, Any, Any] | None:
    """(serial, pos, quat) for a valid, well-formed tracker; ``None`` to skip it.

    A single malformed tracker (missing serial/pos/quat) must NOT crash the whole packet — one
    bad entry would otherwise drop every good tracker in the same UDP frame (and kill the Mac
    monitor loop). Structural errors (packet/tracker not an object) still raise via ``_trackers``.
    """
    if not isinstance(tracker, dict):
        raise ValueError("tracker entry must be a JSON object")
    if not tracker.get("valid"):
        return None
    serial, pos, quat = tracker.get("serial"), tracker.get("pos"), tracker.get("quat")
    if serial is None or pos is None or quat is None:
        return None  # malformed: skip, don't KeyError
    return str(serial), pos, quat


def valid_tracker_serials(packet: Any) -> list[str]:
    return [serial for serial, update in tracker_updates(packet).items() if update.pose is not None]


def pose_from_packet(packet: Any, tracker_serial: str | None = None) -> VivePose | None:
    """First valid tracker (or the one matching ``tracker_serial``) as a ROS-frame VivePose."""
    for serial, update in tracker_updates(packet).items():
        if update.pose is None:
            continue
        if tracker_serial is not None and serial != tracker_serial:
            continue
        position, orientation = update.pose
        return VivePose(serial=serial, position=position, orientation_xyzw=orientation)
    return None


def parse_trackers(packet: Any) -> dict[str, TrackerPose]:
    """Every valid tracker in the packet as {serial: (pos, quat_xyzw)} in the ROS/body frame.

    Valid-but-malformed trackers (missing pos/quat/serial) are skipped, not fatal, so one bad
    entry never drops the rest of the packet.
    """
    return {
        serial: update.pose
        for serial, update in tracker_updates(packet).items()
        if update.pose is not None
    }


def tracker_updates(packet: Any) -> dict[str, TrackerUpdate]:
    """Parse valid poses and explicit invalidations from either supported Vive backend."""
    trackers = _trackers(packet)
    frame = _packet_frame(packet)
    convert = openvr_pose_to_ros if frame == "openvr" else libsurvive_pose_to_ros
    updates: dict[str, TrackerUpdate] = {}
    for tracker in trackers:
        if not isinstance(tracker, dict):
            raise ValueError("tracker entry must be a JSON object")
        serial_value = tracker.get("serial")
        if serial_value is None:
            continue
        serial = str(serial_value)
        if not tracker.get("valid"):
            updates[serial] = TrackerUpdate(pose=None)
            continue
        fields = _tracker_pose_fields(tracker)
        if fields is None:
            continue
        serial, pos, quat = fields
        sample_age = tracker.get("pose_age_s", 0.0)
        if isinstance(sample_age, bool) or not isinstance(sample_age, (int, float)):
            raise ValueError("pose_age_s must be a non-negative finite number")
        sample_age_s = float(sample_age)
        if not np.isfinite(sample_age_s) or sample_age_s < 0.0:
            raise ValueError("pose_age_s must be a non-negative finite number")
        updates[serial] = TrackerUpdate(
            pose=convert(pos, quat),
            sample_age_s=sample_age_s,
        )
    return updates


class MultiViveZeroer:
    """Tare every UMI to ONE shared reference frame captured from a chosen UMI at a zero instant.

    Uses one shared reference pose rather than zeroing each stream independently. After
    ``request_zero(serial)``, the next :meth:`relative` snapshots that UMI's pose as the origin
    ``F0`` and from then on returns EVERY UMI's 6-DoF pose as ``F0^-1 . pose`` — so the reference
    reads 0 at the zero instant and shows its own motion after, while other UMIs are expressed in
    the frame the reference occupied at zero (position and axes). Before the first zero, absolute
    poses are returned.
    """

    def __init__(self) -> None:
        self._frame: tuple[np.ndarray, np.ndarray] | None = None  # (p0, q0)
        self._ref_serial: str | None = None
        self._request: str | None = None

    @property
    def reference(self) -> str | None:
        return self._ref_serial

    def request_zero(self, serial: str) -> None:
        """Capture ``serial``'s next pose as the shared origin for all UMIs."""
        self._request = serial

    def relative(self, poses: dict[str, TrackerPose]) -> dict[str, Pose6]:
        """Map {serial: (pos, quat_xyzw)} to {serial: (x, y, z, roll, pitch, yaw)}."""
        if self._request is not None and self._request in poses:
            p0, q0 = poses[self._request]
            self._frame = (np.asarray(p0, dtype=np.float64), _normalize(np.asarray(q0)))
            self._ref_serial = self._request
            self._request = None

        out: dict[str, Pose6] = {}
        if self._frame is None:
            for serial, (p, q) in poses.items():
                out[serial] = _to_pose6(np.asarray(p, dtype=np.float64), _normalize(np.asarray(q)))
            return out

        p0, q0_inv = self._frame[0], _conjugate(self._frame[1])
        for serial, (p, q) in poses.items():
            rel_p = _rotate(q0_inv, np.asarray(p, dtype=np.float64) - p0)
            rel_q = _multiply(q0_inv, _normalize(np.asarray(q)))
            out[serial] = _to_pose6(rel_p, rel_q)
        return out


def euler_deg(q: np.ndarray) -> tuple[float, float, float]:
    """ZYX roll/pitch/yaw (deg) from a normalized quat [x,y,z,w], gimbal-lock-safe for display.

    Near |pitch| = 90 deg the ZYX roll and yaw are degenerate and swing wildly on tiny quaternion
    noise even when the orientation is steady (only roll +- yaw is well-defined). There we fold them
    into a single well-defined angle (roll = 0, everything in yaw) so the live readout stays stable
    instead of jumping, rather than the naive arctan2 form used for the LCD (ros.quat_to_euler_deg).
    """
    x, y, z, w = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    sinp = 2.0 * (w * y - z * x)
    if sinp > 0.99999:  # pitch ~ +90 deg (north pole)
        return 0.0, 90.0, float(np.degrees(-2.0 * np.arctan2(x, w)))
    if sinp < -0.99999:  # pitch ~ -90 deg (south pole)
        return 0.0, -90.0, float(np.degrees(2.0 * np.arctan2(x, w)))
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(sinp)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw))


def _to_pose6(p: np.ndarray, q: np.ndarray) -> Pose6:
    roll, pitch, yaw = euler_deg(q)
    return float(p[0]), float(p[1]), float(p[2]), roll, pitch, yaw

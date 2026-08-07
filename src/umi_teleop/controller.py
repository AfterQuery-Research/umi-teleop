"""Fail-closed state and geometry for dual-UMI Vive teleoperation.

This module is deliberately hardware-free.  The launcher owns I2RT robots and inverse
kinematics; tests can exercise session changes, zeroing, stale tracking, button counters, and
workspace limits without touching CAN.
"""

from __future__ import annotations

import math
import socket
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from umi_teleop.protocol import (
    ARM_NAMES,
    decode_teleop_packet,
)

ZERO_FRAME_AXIS_MAPS = {
    "identity": np.eye(3, dtype=np.float64),
    # UMI handle axes observed on hardware:
    # physical forward produced robot up, physical up produced robot forward,
    # and physical left produced robot right. This maps the zeroed UMI handle
    # frame back to the desired robot command frame.
    "umi-handle": np.array(
        [[0.0, 0.0, 1.0], [0.0, -1.0, 0.0], [1.0, 0.0, 0.0]],
        dtype=np.float64,
    ),
}


def decode_packet(data: bytes) -> dict[str, Any]:
    """Strictly decode one datagram from the UMI source."""
    return decode_teleop_packet(data)


class UdpTeleopReceiver:
    """Non-blocking latest-packet receiver with arrival age measured on this host."""

    def __init__(self, host: str = "0.0.0.0", port: int = 9140):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.port = self.sock.getsockname()[1]
        self.sock.setblocking(False)
        self.latest: dict[str, Any] | None = None
        self.received_at: float | None = None
        self.bad_packets = 0

    def poll(self) -> tuple[dict[str, Any] | None, float | None]:
        while True:
            try:
                data, _ = self.sock.recvfrom(65_535)
            except BlockingIOError:
                break
            try:
                packet = decode_packet(data)
            except (ValueError, TypeError, UnicodeDecodeError):
                self.bad_packets += 1
                continue
            if (
                self.latest is not None
                and packet["session_id"] == self.latest["session_id"]
                and packet["seq"] <= self.latest["seq"]
            ):
                continue
            self.latest = packet
            self.received_at = time.monotonic()
        age = None if self.received_at is None else time.monotonic() - self.received_at
        return self.latest, age

    def close(self) -> None:
        self.sock.close()


def quat_to_matrix(quat_xyzw: np.ndarray) -> np.ndarray:
    quat = np.asarray(quat_xyzw, dtype=np.float64)
    if quat.shape != (4,) or not np.isfinite(quat).all():
        raise ValueError("quaternion must have four finite values")
    norm = float(np.linalg.norm(quat))
    if norm < 1e-9:
        raise ValueError("zero quaternion")
    x, y, z, w = quat / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def pose_matrix(pose_xyz_xyzw: list[float], tip_offset_m: np.ndarray | None = None) -> np.ndarray:
    pose = np.asarray(pose_xyz_xyzw, dtype=np.float64)
    if pose.shape != (7,) or not np.isfinite(pose).all():
        raise ValueError("pose must have seven finite values")
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = quat_to_matrix(pose[3:])
    out[:3, 3] = pose[:3]
    if tip_offset_m is not None:
        offset = np.asarray(tip_offset_m, dtype=np.float64)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("tip offset must have three finite values")
        out[:3, 3] += out[:3, :3] @ offset
    return out


def rotation_angle(rotation: np.ndarray) -> float:
    cos_angle = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.arccos(cos_angle))


ROTATION_IDENTITY_EPS_RAD = 1e-7


def limit_rotation_angle(rotation: np.ndarray, max_angle_rad: float) -> np.ndarray:
    """Project a proper rotation onto a maximum angle from identity."""
    checked = checked_rotation_matrix(rotation)
    if not math.isfinite(max_angle_rad) or not 0.0 <= max_angle_rad <= math.pi:
        raise ValueError("max rotation angle must be between zero and pi")
    angle = rotation_angle(checked)
    if angle <= max_angle_rad:
        return checked
    if max_angle_rad == 0.0 or angle < ROTATION_IDENTITY_EPS_RAD:
        return np.eye(3, dtype=np.float64)

    if math.isclose(angle, math.pi, abs_tol=1e-6):
        eigenvalues, eigenvectors = np.linalg.eig(checked)
        axis_index = int(np.argmin(np.abs(eigenvalues - 1.0)))
        axis = np.real(eigenvectors[:, axis_index])
    else:
        axis = np.array(
            [
                checked[2, 1] - checked[1, 2],
                checked[0, 2] - checked[2, 0],
                checked[1, 0] - checked[0, 1],
            ],
            dtype=np.float64,
        ) / (2.0 * math.sin(angle))
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-9:
        raise ValueError("rotation axis is numerically unstable")
    x, y, z = axis / axis_norm
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)
    return (
        np.eye(3, dtype=np.float64)
        + math.sin(max_angle_rad) * skew
        + (1.0 - math.cos(max_angle_rad)) * (skew @ skew)
    )


def scale_rotation_angle(rotation: np.ndarray, scale: float) -> np.ndarray:
    """Scale a proper rotation's angle while preserving its axis."""
    checked = checked_rotation_matrix(rotation)
    if not math.isfinite(scale) or scale < 0.0:
        raise ValueError("rotation scale must be finite and non-negative")
    angle = rotation_angle(checked)
    if angle < ROTATION_IDENTITY_EPS_RAD:
        return np.eye(3, dtype=np.float64)
    if scale == 1.0:
        return checked
    target_angle = min(math.pi, angle * scale)
    if scale <= 1.0:
        return limit_rotation_angle(checked, target_angle)
    if math.isclose(angle, math.pi, abs_tol=1e-6):
        eigenvalues, eigenvectors = np.linalg.eig(checked)
        axis = np.real(eigenvectors[:, int(np.argmin(np.abs(eigenvalues - 1.0)))])
    else:
        axis = np.array(
            [
                checked[2, 1] - checked[1, 2],
                checked[0, 2] - checked[2, 0],
                checked[1, 0] - checked[0, 1],
            ],
            dtype=np.float64,
        ) / (2.0 * math.sin(angle))
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)
    return (
        np.eye(3, dtype=np.float64)
        + math.sin(target_angle) * skew
        + (1.0 - math.cos(target_angle)) * (skew @ skew)
    )


def filter_rotation(previous: np.ndarray, target: np.ndarray, alpha: float) -> np.ndarray:
    """Interpolate from previous to target on SO(3)."""
    if alpha >= 1.0:
        return checked_rotation_matrix(target)
    if alpha <= 0.0:
        return checked_rotation_matrix(previous)
    relative = checked_rotation_matrix(target) @ checked_rotation_matrix(previous).T
    return scale_rotation_angle(relative, alpha) @ previous


def checked_rotation_matrix(rotation: np.ndarray, name: str = "rotation") -> np.ndarray:
    """Return a finite, proper 3-D rotation matrix or raise ``ValueError``."""
    out = np.asarray(rotation, dtype=np.float64)
    if out.shape != (3, 3) or not np.isfinite(out).all():
        raise ValueError(f"{name} must be a finite 3x3 matrix")
    if not np.allclose(out.T @ out, np.eye(3), atol=1e-6, rtol=0.0):
        raise ValueError(f"{name} must be orthonormal")
    if not math.isclose(float(np.linalg.det(out)), 1.0, abs_tol=1e-6):
        raise ValueError(f"{name} must be a proper rotation")
    return out.copy()


def world_frame_delta(
    zero_pose: np.ndarray,
    current_pose: np.ndarray,
    input_to_robot_rotation: np.ndarray | None = None,
) -> np.ndarray:
    """Map a pose change from the Vive world frame into the robot base frame.

    Translation is the world-frame displacement ``p - p0``. Orientation is the
    world-frame rotation ``R R0.T``. This deliberately avoids ``inv(T0) @ T``, which
    expresses motion in the tracker's zero-local axes and makes physical up depend on how
    the handheld UMI happened to be oriented when ZERO was pressed.
    """
    zero = np.asarray(zero_pose, dtype=np.float64)
    current = np.asarray(current_pose, dtype=np.float64)
    if zero.shape != (4, 4) or current.shape != (4, 4):
        raise ValueError("zero_pose and current_pose must be 4x4 matrices")
    if not np.isfinite(zero).all() or not np.isfinite(current).all():
        raise ValueError("zero_pose and current_pose must be finite")
    basis = checked_rotation_matrix(
        np.eye(3) if input_to_robot_rotation is None else input_to_robot_rotation,
        "input_to_robot_rotation",
    )
    zero_rotation = checked_rotation_matrix(zero[:3, :3], "zero_pose rotation")
    current_rotation = checked_rotation_matrix(current[:3, :3], "current_pose rotation")

    delta = np.eye(4, dtype=np.float64)
    delta[:3, 3] = basis @ (current[:3, 3] - zero[:3, 3])
    input_world_rotation = current_rotation @ zero_rotation.T
    delta[:3, :3] = basis @ input_world_rotation @ basis.T
    return delta


@dataclass(frozen=True)
class GripperMap:
    closed_mm: float
    open_mm: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.closed_mm) or not math.isfinite(self.open_mm):
            raise ValueError("gripper widths must be finite")
        if abs(self.open_mm - self.closed_mm) < 1e-6:
            raise ValueError("open and closed UMI widths must differ")

    def normalized(self, width_mm: float) -> float:
        value = (float(width_mm) - self.closed_mm) / (self.open_mm - self.closed_mm)
        return float(np.clip(value, 0.0, 1.0))


@dataclass(frozen=True)
class TeleopLimits:
    max_packet_age_s: float = 0.25
    max_pose_age_s: float = 0.25
    max_width_age_s: float = 0.75
    max_translation_m: float = 0.50
    max_rotation_rad: float = math.radians(100.0)
    max_zero_translation_m: float = 0.75
    max_zero_rotation_rad: float = math.radians(150.0)
    position_scale: float = 1.0
    rotation_scale: float = 1.0
    translation_filter_hz: float = 0.0
    rotation_filter_hz: float = 0.0
    translation_deadband_m: float = 0.0
    rotation_deadband_rad: float = 0.0
    require_width_to_arm: bool = True


@dataclass
class StepResult:
    armed: bool
    zeroed: bool
    quit: bool = False
    zero_captured: bool = False
    reason: str = ""
    targets: dict[str, np.ndarray] | None = None
    grippers: dict[str, float | None] | None = None
    input_deltas: dict[str, np.ndarray] | None = None
    command_deltas: dict[str, np.ndarray] | None = None
    envelope_clamps: dict[str, tuple[str, ...]] | None = None


class TeleopStateMachine:
    """Consume repeated button counters and produce bounded Cartesian targets."""

    def __init__(
        self,
        gripper_maps: dict[str, GripperMap],
        limits: TeleopLimits | None = None,
        tip_offsets_m: dict[str, np.ndarray] | None = None,
        input_to_robot_rotation: np.ndarray | None = None,
        zero_frame_source: str = "fixed",
        zero_frame_axis_map: str = "identity",
    ):
        if set(gripper_maps) != set(ARM_NAMES):
            raise ValueError("gripper_maps must contain left and right")
        if zero_frame_source not in ("fixed", "left", "right", "each"):
            raise ValueError("zero_frame_source must be fixed, left, right, or each")
        if zero_frame_axis_map not in ZERO_FRAME_AXIS_MAPS:
            raise ValueError(
                "zero_frame_axis_map must be one of " + ", ".join(sorted(ZERO_FRAME_AXIS_MAPS))
            )
        self.gripper_maps = gripper_maps
        self.limits = limits or TeleopLimits()
        if not math.isfinite(self.limits.max_translation_m) or self.limits.max_translation_m < 0.0:
            raise ValueError("max translation must be finite and nonnegative")
        if (
            not math.isfinite(self.limits.max_rotation_rad)
            or not 0.0 <= self.limits.max_rotation_rad <= math.pi
        ):
            raise ValueError("max rotation must be between zero and pi")
        if (
            not math.isfinite(self.limits.max_zero_translation_m)
            or self.limits.max_zero_translation_m <= 0.0
        ):
            raise ValueError("maximum ZERO residual translation must be finite and positive")
        if (
            not math.isfinite(self.limits.max_zero_rotation_rad)
            or not 0.0 < self.limits.max_zero_rotation_rad <= math.pi
        ):
            raise ValueError("maximum ZERO residual rotation must be between zero and pi")
        for name, value in (
            ("position scale", self.limits.position_scale),
            ("rotation scale", self.limits.rotation_scale),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        for name, value in (
            ("translation filter", self.limits.translation_filter_hz),
            ("rotation filter", self.limits.rotation_filter_hz),
            ("translation deadband", self.limits.translation_deadband_m),
            ("rotation deadband", self.limits.rotation_deadband_rad),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        self.tip_offsets_m = {
            name: np.asarray((tip_offsets_m or {}).get(name, np.zeros(3)), dtype=np.float64)
            for name in ARM_NAMES
        }
        self.input_to_robot_rotation = checked_rotation_matrix(
            np.eye(3) if input_to_robot_rotation is None else input_to_robot_rotation,
            "input_to_robot_rotation",
        )
        self.zero_frame_source = zero_frame_source
        self.zero_frame_axis_map = zero_frame_axis_map
        self.zero_frame_axis_rotation = checked_rotation_matrix(
            ZERO_FRAME_AXIS_MAPS[zero_frame_axis_map],
            f"zero_frame_axis_map {zero_frame_axis_map}",
        )
        self.session_id: str | None = None
        self.last_seq = -1
        self.zero_id = 0
        self.toggle_id = 0
        self.cancel_id = 0
        self.quit_id = 0
        self.zeroed = False
        self.armed = False
        self.reason = "waiting for source session"
        self.input_zero: dict[str, np.ndarray] = {}
        self.robot_zero: dict[str, np.ndarray] = {}
        self.zero_input_to_robot_rotation: dict[str, np.ndarray] = {}
        self.filtered_deltas: dict[str, np.ndarray] = {}
        self._last_filter_at: float | None = None

    def disarm(self, reason: str) -> None:
        self.armed = False
        self.reason = reason

    def invalidate_zero(self, reason: str) -> None:
        """Require a fresh operator ZERO after an external robot-frame change."""
        self.input_zero.clear()
        self.robot_zero.clear()
        self.zero_input_to_robot_rotation.clear()
        self.filtered_deltas.clear()
        self._last_filter_at = None
        self.zeroed = False
        self.disarm(reason)

    def reject_zero(self, reason: str) -> None:
        """Invalidate a captured zero that the hardware launcher found unsafe."""
        self.invalidate_zero(reason)

    def _new_session(self, packet: dict[str, Any]) -> StepResult:
        self.session_id = packet["session_id"]
        self.last_seq = packet["seq"]
        self.zero_id = packet["zero_id"]
        self.toggle_id = packet["toggle_id"]
        self.cancel_id = packet["cancel_id"]
        self.quit_id = packet["quit_id"]
        self.zeroed = False
        self.input_zero.clear()
        self.robot_zero.clear()
        self.zero_input_to_robot_rotation.clear()
        self.filtered_deltas.clear()
        self._last_filter_at = None
        self.disarm("new source session; tap ZERO")
        return StepResult(armed=False, zeroed=False, reason=self.reason)

    def _zero_frame_rotations(self) -> dict[str, np.ndarray]:
        if set(self.input_zero) != set(ARM_NAMES):
            raise ValueError("input_zero must contain left and right before deriving zero frame")
        if self.zero_frame_source == "fixed":
            frame = self.zero_frame_axis_rotation @ self.input_to_robot_rotation
            return {name: frame.copy() for name in ARM_NAMES}
        if self.zero_frame_source == "each":
            return {
                name: checked_rotation_matrix(
                    self.zero_frame_axis_rotation
                    @ self.input_to_robot_rotation
                    @ self.input_zero[name][:3, :3].T,
                    f"{name} zero frame rotation",
                )
                for name in ARM_NAMES
            }
        reference = self.input_zero[self.zero_frame_source]
        frame = checked_rotation_matrix(
            self.zero_frame_axis_rotation @ self.input_to_robot_rotation @ reference[:3, :3].T,
            f"{self.zero_frame_source} zero frame rotation",
        )
        return {name: frame.copy() for name in ARM_NAMES}

    def _fresh_poses(self, packet: dict[str, Any]) -> bool:
        return all(
            packet["arms"][name]["pose"] is not None
            and packet["arms"][name]["pose_age_s"] <= self.limits.max_pose_age_s
            for name in ARM_NAMES
        )

    def _fresh_widths(self, packet: dict[str, Any]) -> bool:
        return all(
            packet["arms"][name]["width_mm"] is not None
            and packet["arms"][name]["width_age_s"] <= self.limits.max_width_age_s
            for name in ARM_NAMES
        )

    def _input_deltas(self, packet: dict[str, Any]) -> dict[str, np.ndarray]:
        return {
            name: world_frame_delta(
                self.input_zero[name],
                pose_matrix(packet["arms"][name]["pose"], self.tip_offsets_m[name]),
                self.zero_input_to_robot_rotation[name],
            )
            for name in ARM_NAMES
        }

    def _zero_discontinuity(self, input_deltas: dict[str, np.ndarray]) -> str | None:
        violations: list[str] = []
        for name, delta in input_deltas.items():
            distance = float(np.linalg.norm(delta[:3, 3]))
            angle = rotation_angle(delta[:3, :3])
            if distance > self.limits.max_zero_translation_m:
                violations.append(
                    f"{name} translation {distance:.3f}m>{self.limits.max_zero_translation_m:.3f}m"
                )
            if angle > self.limits.max_zero_rotation_rad:
                violations.append(
                    f"{name} rotation {math.degrees(angle):.1f}deg"
                    f">{math.degrees(self.limits.max_zero_rotation_rad):.1f}deg"
                )
        if not violations:
            return None
        return "tracking frame changed after ZERO (" + "; ".join(violations) + "); tap ZERO again"

    def step(
        self,
        packet: dict[str, Any],
        packet_age_s: float,
        current_robot_poses: dict[str, np.ndarray],
        now_s: float | None = None,
    ) -> StepResult:
        now_s = time.monotonic() if now_s is None else now_s
        if self.session_id != packet["session_id"]:
            return self._new_session(packet)
        # This check must precede the sequence check: the launcher calls step repeatedly with the
        # latest packet, so its arrival age can become unsafe even when no new sequence arrives.
        if packet_age_s > self.limits.max_packet_age_s:
            self.disarm(f"telemetry stale ({packet_age_s:.3f}s)")
            return StepResult(False, self.zeroed, reason=self.reason)
        if packet["seq"] <= self.last_seq:
            return StepResult(self.armed, self.zeroed, reason="old packet ignored")
        self.last_seq = packet["seq"]

        quit_requested = packet["quit_id"] > self.quit_id
        cancelled = packet["cancel_id"] > self.cancel_id
        zero_requested = packet["zero_id"] > self.zero_id
        toggle_changed = packet["toggle_id"] > self.toggle_id
        self.quit_id = max(self.quit_id, packet["quit_id"])
        self.cancel_id = max(self.cancel_id, packet["cancel_id"])
        self.zero_id = max(self.zero_id, packet["zero_id"])
        self.toggle_id = max(self.toggle_id, packet["toggle_id"])

        if quit_requested:
            self.disarm("source quit requested")
            return StepResult(False, self.zeroed, quit=True, reason=self.reason)
        if cancelled:
            self.disarm("cancel button")
        if not packet["requested_enabled"]:
            self.disarm("source disabled")

        zero_captured = False
        if zero_requested:
            self.disarm("zero captured; tap enable")
            if not self._fresh_poses(packet):
                self.zeroed = False
                self.reason = "ZERO rejected: both fresh Vive poses are required"
                return StepResult(False, False, reason=self.reason)
            if set(current_robot_poses) != set(ARM_NAMES):
                raise ValueError("current_robot_poses must contain left and right")
            self.input_zero = {
                name: pose_matrix(packet["arms"][name]["pose"], self.tip_offsets_m[name])
                for name in ARM_NAMES
            }
            self.robot_zero = {
                name: np.asarray(current_robot_poses[name], dtype=np.float64).copy()
                for name in ARM_NAMES
            }
            self.zero_input_to_robot_rotation = self._zero_frame_rotations()
            self.zeroed = True
            zero_captured = True
            self.filtered_deltas = {name: np.eye(4, dtype=np.float64) for name in ARM_NAMES}
            self._last_filter_at = now_s

        input_deltas: dict[str, np.ndarray] | None = None
        if toggle_changed:
            if not packet["requested_enabled"]:
                self.disarm("enable toggled off")
            elif not self.zeroed:
                self.disarm("enable rejected: tap ZERO first")
            elif not self._fresh_poses(packet):
                self.disarm("enable rejected: Vive pose stale")
            elif self.limits.require_width_to_arm and not self._fresh_widths(packet):
                self.disarm("enable rejected: ArUco width stale")
            else:
                input_deltas = self._input_deltas(packet)
                discontinuity = self._zero_discontinuity(input_deltas)
                if discontinuity is not None:
                    self.invalidate_zero(discontinuity)
                else:
                    self.armed = True
                    self.reason = "enabled"

        if not self.armed:
            return StepResult(False, self.zeroed, zero_captured=zero_captured, reason=self.reason)
        if not self._fresh_poses(packet):
            self.disarm("Vive tracking stale; re-enable after recovery")
            return StepResult(False, self.zeroed, reason=self.reason)
        if self.limits.require_width_to_arm and not self._fresh_widths(packet):
            self.disarm("Pi camera or ArUco width stale; re-enable after recovery")
            return StepResult(False, self.zeroed, reason=self.reason)

        if input_deltas is None:
            input_deltas = self._input_deltas(packet)
        discontinuity = self._zero_discontinuity(input_deltas)
        if discontinuity is not None:
            self.invalidate_zero(discontinuity)
            return StepResult(False, False, reason=self.reason)

        dt = 0.0 if self._last_filter_at is None else max(0.0, now_s - self._last_filter_at)
        self._last_filter_at = now_s

        def alpha(cutoff_hz: float) -> float:
            if cutoff_hz <= 0.0 or dt <= 0.0:
                return 1.0
            return 1.0 - math.exp(-2.0 * math.pi * cutoff_hz * dt)

        translation_alpha = alpha(self.limits.translation_filter_hz)
        rotation_alpha = alpha(self.limits.rotation_filter_hz)

        targets: dict[str, np.ndarray] = {}
        grippers: dict[str, float | None] = {}
        command_deltas: dict[str, np.ndarray] = {}
        envelope_clamps: dict[str, tuple[str, ...]] = {}
        for name in ARM_NAMES:
            delta = input_deltas[name]
            command_delta = delta.copy()
            translation = command_delta[:3, 3]
            translation_norm = float(np.linalg.norm(translation))
            if translation_norm <= self.limits.translation_deadband_m:
                command_delta[:3, 3] = 0.0
            elif self.limits.translation_deadband_m > 0.0:
                command_delta[:3, 3] *= (
                    translation_norm - self.limits.translation_deadband_m
                ) / translation_norm
            command_angle = rotation_angle(command_delta[:3, :3])
            if command_angle <= self.limits.rotation_deadband_rad:
                command_delta[:3, :3] = np.eye(3, dtype=np.float64)
            else:
                active_angle = command_angle - self.limits.rotation_deadband_rad
                command_delta[:3, :3] = scale_rotation_angle(
                    command_delta[:3, :3],
                    active_angle / command_angle * self.limits.rotation_scale,
                )
            previous = self.filtered_deltas.get(name, np.eye(4, dtype=np.float64))
            filtered = np.eye(4, dtype=np.float64)
            filtered[:3, 3] = previous[:3, 3] + translation_alpha * (
                command_delta[:3, 3] - previous[:3, 3]
            )
            filtered[:3, :3] = filter_rotation(
                previous[:3, :3], command_delta[:3, :3], rotation_alpha
            )
            self.filtered_deltas[name] = filtered
            command_delta = filtered
            clamp_reasons = []
            translation_norm = float(np.linalg.norm(command_delta[:3, 3]))
            angle = rotation_angle(command_delta[:3, :3])
            if translation_norm > self.limits.max_translation_m:
                command_delta[:3, 3] *= self.limits.max_translation_m / translation_norm
                clamp_reasons.append(
                    f"translation {translation_norm:.3f}m->{self.limits.max_translation_m:.3f}m"
                )
            if angle > self.limits.max_rotation_rad:
                command_delta[:3, :3] = limit_rotation_angle(
                    command_delta[:3, :3], self.limits.max_rotation_rad
                )
                clamp_reasons.append(
                    f"rotation {math.degrees(angle):.1f}deg"
                    f"->{math.degrees(self.limits.max_rotation_rad):.1f}deg"
                )
            if clamp_reasons:
                envelope_clamps[name] = tuple(clamp_reasons)
            target = self.robot_zero[name].copy()
            target[:3, 3] += command_delta[:3, 3] * self.limits.position_scale
            target[:3, :3] = command_delta[:3, :3] @ target[:3, :3]
            targets[name] = target
            command_deltas[name] = command_delta
            arm = packet["arms"][name]
            if arm["width_mm"] is None or arm["width_age_s"] > self.limits.max_width_age_s:
                grippers[name] = None
            else:
                grippers[name] = self.gripper_maps[name].normalized(arm["width_mm"])
        if envelope_clamps:
            self.reason = "input_clamped: " + "; ".join(
                f"{name} " + ", ".join(reasons) for name, reasons in envelope_clamps.items()
            )
        else:
            self.reason = "enabled"
        return StepResult(
            True,
            self.zeroed,
            zero_captured=zero_captured,
            reason=self.reason,
            targets=targets,
            grippers=grippers,
            input_deltas=input_deltas,
            command_deltas=command_deltas,
            envelope_clamps=envelope_clamps or None,
        )

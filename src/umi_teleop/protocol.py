"""Small, fail-closed fused telemetry protocol for UMI-to-YAM teleoperation.

On native Linux, aq-yam fuses local libsurvive poses with Raspberry Pi gripper and GPIO telemetry
before sending this controller packet over loopback. This module keeps that assembly independent
of ROS and of the recorder.

Button counters are repeated in every datagram.  A dropped UDP packet therefore cannot lose a
zero, cancel, or quit request; the YAM controller observes the next counter value instead.
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
import uuid
from contextlib import suppress
from dataclasses import dataclass, field
from typing import Any

from umi_teleop.types import Event
from umi_teleop.vive import tracker_updates

PROTOCOL_VERSION = 1
ARM_NAMES = ("left", "right")
MAX_DATAGRAM_BYTES = 16_384
RECEIVE_BUFFER_BYTES = 1 << 20


def _finite_number(value: Any, name: str, *, minimum: float | None = None) -> float:
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


def _validate_arm(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"arms.{name} must be an object")
    pose = value.get("pose")
    pose_age = value.get("pose_age_s")
    if pose is None:
        if pose_age is not None:
            raise ValueError(f"arms.{name}.pose_age_s must be null when pose is null")
    else:
        if not isinstance(pose, list) or len(pose) != 7:
            raise ValueError(f"arms.{name}.pose must contain xyz + quaternion xyzw")
        pose = [_finite_number(v, f"arms.{name}.pose") for v in pose]
        qnorm = math.sqrt(sum(v * v for v in pose[3:]))
        if qnorm < 1e-6 or abs(qnorm - 1.0) > 1e-3:
            raise ValueError(f"arms.{name}.pose quaternion must be normalized")
        pose_age = _finite_number(pose_age, f"arms.{name}.pose_age_s", minimum=0.0)

    width = value.get("width_mm")
    width_age = value.get("width_age_s")
    if width is None:
        if width_age is not None:
            raise ValueError(f"arms.{name}.width_age_s must be null when width_mm is null")
    else:
        width = _finite_number(width, f"arms.{name}.width_mm")
        width_age = _finite_number(width_age, f"arms.{name}.width_age_s", minimum=0.0)
    return {
        "serial": str(value.get("serial", "")),
        "pose": pose,
        "pose_age_s": pose_age,
        "width_mm": width,
        "width_age_s": width_age,
    }


def decode_teleop_packet(data: bytes) -> dict[str, Any]:
    """Decode and strictly validate one Pi-to-YAM telemetry datagram."""
    if not data or len(data) > MAX_DATAGRAM_BYTES:
        raise ValueError("teleop datagram is empty or too large")
    packet = json.loads(data.decode("utf-8"))
    if not isinstance(packet, dict):
        raise ValueError("teleop packet must be a JSON object")
    if packet.get("version") != PROTOCOL_VERSION:
        raise ValueError(f"unsupported teleop protocol version {packet.get('version')!r}")
    session_id = packet.get("session_id")
    if not isinstance(session_id, str) or not session_id or len(session_id) > 128:
        raise ValueError("session_id must be a non-empty string of at most 128 characters")
    if not isinstance(packet.get("requested_enabled"), bool):
        raise ValueError("requested_enabled must be boolean")
    arms = packet.get("arms")
    if not isinstance(arms, dict):
        raise ValueError("arms must be an object")
    return {
        "version": PROTOCOL_VERSION,
        "session_id": session_id,
        "seq": _counter(packet.get("seq"), "seq"),
        "requested_enabled": packet["requested_enabled"],
        "zero_id": _counter(packet.get("zero_id"), "zero_id"),
        "toggle_id": _counter(packet.get("toggle_id"), "toggle_id"),
        "cancel_id": _counter(packet.get("cancel_id"), "cancel_id"),
        "quit_id": _counter(packet.get("quit_id"), "quit_id"),
        "arms": {name: _validate_arm(arms.get(name), name) for name in ARM_NAMES},
    }


def encode_teleop_packet(packet: dict[str, Any]) -> bytes:
    """Validate and encode a telemetry packet using a compact deterministic JSON form."""
    clean = decode_teleop_packet(
        json.dumps(packet, allow_nan=False, separators=(",", ":")).encode("utf-8")
    )
    return json.dumps(clean, allow_nan=False, separators=(",", ":")).encode("utf-8")


@dataclass
class ButtonState:
    """Persistent command state derived from the two Pi buttons."""

    session_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    requested_enabled: bool = False
    zero_id: int = 0
    toggle_id: int = 0
    cancel_id: int = 0
    quit_id: int = 0
    quitting: bool = False

    def handle(self, event: Event) -> None:
        if event is Event.ZERO:
            self.zero_id += 1
            self.requested_enabled = False
        elif event in (Event.TOGGLE, Event.START, Event.STOP):
            self.toggle_id += 1
            if event is Event.START:
                self.requested_enabled = True
            elif event is Event.STOP:
                self.requested_enabled = False
            else:
                self.requested_enabled = not self.requested_enabled
        elif event is Event.CANCEL:
            self.cancel_id += 1
            self.requested_enabled = False
        elif event is Event.QUIT:
            self.quit_id += 1
            self.requested_enabled = False
            self.quitting = True

    def safety_disarm(self) -> None:
        """Drop an enable request when the source itself sees stale tracking."""
        if self.requested_enabled:
            self.cancel_id += 1
            self.requested_enabled = False

    def packet(
        self,
        seq: int,
        poses: dict[str, dict[str, Any]],
        widths: dict[str, dict[str, Any]],
    ) -> dict[str, Any]:
        arms: dict[str, dict[str, Any]] = {}
        for name in ARM_NAMES:
            pose = poses.get(name, {})
            width = widths.get(name, {})
            arms[name] = {
                "serial": pose.get("serial", ""),
                "pose": pose.get("pose"),
                "pose_age_s": pose.get("age_s"),
                "width_mm": width.get("width_mm"),
                "width_age_s": width.get("age_s"),
            }
        return {
            "version": PROTOCOL_VERSION,
            "session_id": self.session_id,
            "seq": seq,
            "requested_enabled": self.requested_enabled,
            "zero_id": self.zero_id,
            "toggle_id": self.toggle_id,
            "cancel_id": self.cancel_id,
            "quit_id": self.quit_id,
            "arms": arms,
        }


@dataclass
class EnableRequestGuard:
    """Delay arming and suppress noisy re-enable attempts after a disarm.

    GPIO contact noise can make one physical hold look like multiple tap releases. A clean
    enable therefore has to remain the only event for ``delay_s``. Any disarm starts a longer
    cooldown during which release bounce is ignored. Disable, cancel, zero, and quit remain
    immediate; only the transition to enabled is delayed.
    """

    delay_s: float = 3.0
    cooldown_s: float = 5.0
    _pending_until: float | None = field(default=None, init=False)
    _blocked_until: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        if self.delay_s <= 0 or self.cooldown_s <= 0:
            raise ValueError("enable delay and cooldown must be positive")

    @property
    def pending(self) -> bool:
        return self._pending_until is not None

    def _block(self, now: float) -> None:
        self._pending_until = None
        self._blocked_until = max(self._blocked_until, now + self.cooldown_s)

    def force_block(self, now: float) -> None:
        """Cancel a pending enable and suppress bounce after an external safety disarm."""
        self._block(now)

    def handle(self, state: ButtonState, event: Event, now: float) -> str:
        """Apply a raw button event, returning a short diagnostic action."""
        if event is Event.START and state.requested_enabled:
            return "already enabled"
        wants_enable = event is Event.START or (
            event is Event.TOGGLE and not state.requested_enabled
        )
        if wants_enable:
            if now < self._blocked_until:
                return "enable ignored during disarm cooldown"
            if self._pending_until is not None:
                self._block(now)
                return "repeated enable gesture rejected"
            self._pending_until = now + self.delay_s
            return f"enable pending for {self.delay_s:.1f}s"

        self._pending_until = None
        state.handle(event)
        if event in (Event.TOGGLE, Event.STOP, Event.CANCEL):
            self._block(now)
        return "applied immediately"

    def update(
        self,
        state: ButtonState,
        now: float,
        *,
        inputs_fresh: bool,
        stale_description: str = "pose or width telemetry",
    ) -> str | None:
        """Commit a pending enable after its quiet period, or reject it fail-closed."""
        if self._pending_until is None or now < self._pending_until:
            return None
        self._pending_until = None
        if not inputs_fresh:
            state.handle(Event.CANCEL)
            self._block(now)
            return f"enable rejected because {stale_description} is stale"
        state.handle(Event.START)
        return "enable confirmed after quiet period"


@dataclass
class ZeroRequestGuard:
    """Defer ZERO until a button-hand occlusion has cleared and both poses are fresh."""

    settle_s: float = 0.4
    timeout_s: float = 2.0
    _ready_at: float | None = field(default=None, init=False)
    _deadline: float | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        if self.settle_s < 0 or self.timeout_s <= self.settle_s:
            raise ValueError("zero timeout must be positive and longer than its settle time")

    @property
    def pending(self) -> bool:
        return self._ready_at is not None

    def request(self, state: ButtonState, now: float) -> str:
        """Disarm immediately and begin a bounded wait for fresh tracking."""
        state.requested_enabled = False
        self._ready_at = now + self.settle_s
        self._deadline = now + self.timeout_s
        return f"zero pending after {self.settle_s:.1f}s settle"

    def cancel(self) -> None:
        self._ready_at = None
        self._deadline = None

    def update(self, state: ButtonState, now: float, *, poses_fresh: bool) -> str | None:
        """Commit one ZERO counter when fresh, or reject after the bounded timeout."""
        if self._ready_at is None or self._deadline is None:
            return None
        if now >= self._ready_at and poses_fresh:
            self.cancel()
            state.handle(Event.ZERO)
            return "zero confirmed with both fresh poses"
        if now >= self._deadline:
            self.cancel()
            return "zero rejected because both poses did not recover before timeout"
        return None


@dataclass(frozen=True)
class _StampedPose:
    serial: str
    pose: tuple[float, float, float, float, float, float, float]
    received_at: float


class VivePacketReceiver:
    """Receive the Windows sender format without ROS."""

    def __init__(
        self,
        serial_to_arm: dict[str, str],
        host: str = "0.0.0.0",
        port: int = 9001,
    ):
        if set(serial_to_arm.values()) != set(ARM_NAMES):
            raise ValueError("serial_to_arm must map exactly one serial to left and one to right")
        self.serial_to_arm = dict(serial_to_arm)
        self.host = host
        self.port = port
        self.packet_count = 0
        self.bad_packet_count = 0
        self._latest: dict[str, _StampedPose] = {}
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
                raw = json.loads(data.decode("utf-8"))
                updates = tracker_updates(raw)
            except (json.JSONDecodeError, UnicodeDecodeError, TypeError, ValueError):
                self.bad_packet_count += 1
                continue
            received_at = time.monotonic()
            stamped: dict[str, _StampedPose] = {}
            invalid_arms: set[str] = set()
            for serial, update in updates.items():
                arm = self.serial_to_arm.get(serial)
                if arm is None:
                    continue
                if update.pose is None:
                    invalid_arms.add(arm)
                    continue
                position, quat = update.pose
                pose = tuple(float(v) for v in (*position, *quat))
                stamped[arm] = _StampedPose(
                    serial=serial,
                    pose=pose,
                    received_at=received_at - update.sample_age_s,
                )
            with self._lock:
                self.packet_count += 1
                for arm in invalid_arms:
                    self._latest.pop(arm, None)
                self._latest.update(stamped)

    def snapshot(self, now: float | None = None) -> dict[str, dict[str, Any]]:
        now = time.monotonic() if now is None else now
        with self._lock:
            latest = dict(self._latest)
        return {
            name: {
                "serial": sample.serial,
                "pose": list(sample.pose),
                "age_s": max(0.0, now - sample.received_at),
            }
            for name, sample in latest.items()
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


class WidthWorker:
    """Continuously drain one camera and retain the latest successful ArUco width."""

    def __init__(self, camera: Any, detector: Any, detect_hz: float = 15.0):
        if detect_hz <= 0:
            raise ValueError("detect_hz must be positive")
        self.camera = camera
        self.detector = detector
        self.detect_period_s = 1.0 / detect_hz
        self.frames_seen = 0
        self.detections = 0
        self.misses = 0
        self.errors = 0
        self._latest_width: float | None = None
        self._latest_at: float | None = None
        self._latest_frame_at: float | None = None
        self._tags_visible = False
        self._last_error: str | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self.camera.start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        next_detection = 0.0
        while not self._stop.is_set():
            try:
                frames = self.camera.read()
            except Exception as exc:  # noqa: BLE001 - report failure without losing telemetry
                with self._lock:
                    self.errors += 1
                    self._last_error = str(exc)
                    self._tags_visible = False
                time.sleep(0.01)
                continue
            if not frames:
                time.sleep(0.003)
                continue
            now = time.monotonic()
            with self._lock:
                self.frames_seen += len(frames)
                self._latest_frame_at = now
            if now < next_detection:
                continue
            next_detection = now + self.detect_period_s
            try:
                width = self.detector.width_mm(frames[-1].image)
            except Exception as exc:  # noqa: BLE001 - camera processing must not kill telemetry
                with self._lock:
                    self.errors += 1
                    self._last_error = str(exc)
                    self._tags_visible = False
                continue
            if width is None or not math.isfinite(float(width)):
                with self._lock:
                    self.misses += 1
                    self._tags_visible = False
                continue
            with self._lock:
                self._latest_width = float(width)
                self._latest_at = time.monotonic()
                self.detections += 1
                self._tags_visible = True
                self._last_error = None

    def snapshot(
        self,
        now: float | None = None,
        *,
        camera_timeout_s: float = 0.5,
    ) -> dict[str, Any]:
        now = time.monotonic() if now is None else now
        with self._lock:
            width, stamp = self._latest_width, self._latest_at
            frame_stamp = self._latest_frame_at
            status = {
                "tags_visible": self._tags_visible,
                "frames_seen": self.frames_seen,
                "detections": self.detections,
                "misses": self.misses,
                "errors": self.errors,
                "last_error": self._last_error,
            }
        frame_age = None if frame_stamp is None else max(0.0, now - frame_stamp)
        status.update(
            {
                "camera_ok": (
                    self._thread is not None
                    and self._thread.is_alive()
                    and frame_age is not None
                    and frame_age <= camera_timeout_s
                ),
                "frame_age_s": frame_age,
                "width_mm": width,
                "age_s": None if stamp is None else max(0.0, now - stamp),
            }
        )
        return status

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        self.camera.stop()

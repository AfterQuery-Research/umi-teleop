"""Versioned Raspberry Pi sensor telemetry for native Linux teleoperation."""

from __future__ import annotations

import json
import math
import socket
import time
import uuid
from dataclasses import dataclass, field
from typing import Any

from umi_teleop.protocol import ARM_NAMES, MAX_DATAGRAM_BYTES, RECEIVE_BUFFER_BYTES
from umi_teleop.types import Event

PI_PROTOCOL_VERSION = 1
EVENT_NAMES = ("zero", "toggle", "start", "stop", "cancel", "quit")


def _counter(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{name} must be a non-negative integer")
    return value


def _finite(value: Any, name: str, *, minimum: float = 0.0) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or result < minimum:
        raise ValueError(f"{name} must be finite and at least {minimum}")
    return result


def _validate_arm(value: Any, name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"arms.{name} must be an object")
    width = value.get("width_mm")
    width_age = value.get("width_age_s")
    if width is None:
        if width_age is not None:
            raise ValueError(f"arms.{name}.width_age_s must be null when width_mm is null")
    else:
        width = _finite(width, f"arms.{name}.width_mm")
        width_age = _finite(width_age, f"arms.{name}.width_age_s")
    for key in ("tags_visible", "camera_ok"):
        if not isinstance(value.get(key), bool):
            raise ValueError(f"arms.{name}.{key} must be boolean")
    return {
        "width_mm": width,
        "width_age_s": width_age,
        "tags_visible": value["tags_visible"],
        "camera_ok": value["camera_ok"],
        "frame_age_s": (
            None
            if value.get("frame_age_s") is None
            else _finite(value["frame_age_s"], f"arms.{name}.frame_age_s")
        ),
        "frames_seen": _counter(value.get("frames_seen"), f"arms.{name}.frames_seen"),
        "detections": _counter(value.get("detections"), f"arms.{name}.detections"),
        "misses": _counter(value.get("misses"), f"arms.{name}.misses"),
        "errors": _counter(value.get("errors"), f"arms.{name}.errors"),
    }


def decode_pi_packet(data: bytes) -> dict[str, Any]:
    """Decode and strictly validate one Pi sensor datagram."""
    if not data or len(data) > MAX_DATAGRAM_BYTES:
        raise ValueError("Pi datagram is empty or too large")
    packet = json.loads(data.decode("utf-8"))
    if not isinstance(packet, dict):
        raise ValueError("Pi packet must be a JSON object")
    if packet.get("version") != PI_PROTOCOL_VERSION:
        raise ValueError(f"unsupported Pi protocol version {packet.get('version')!r}")
    for key in ("source_id", "session_id"):
        value = packet.get(key)
        if not isinstance(value, str) or not value or len(value) > 128:
            raise ValueError(f"{key} must be a non-empty string of at most 128 characters")
    events = packet.get("events")
    if not isinstance(events, dict):
        raise ValueError("events must be an object")
    arms = packet.get("arms")
    if not isinstance(arms, dict):
        raise ValueError("arms must be an object")
    diagnostics = packet.get("diagnostics", [])
    if not isinstance(diagnostics, list) or not all(isinstance(item, str) for item in diagnostics):
        raise ValueError("diagnostics must be a list of strings")
    return {
        "version": PI_PROTOCOL_VERSION,
        "source_id": packet["source_id"],
        "session_id": packet["session_id"],
        "seq": _counter(packet.get("seq"), "seq"),
        "uptime_s": _finite(packet.get("uptime_s"), "uptime_s"),
        "events": {name: _counter(events.get(name), f"events.{name}") for name in EVENT_NAMES},
        "arms": {name: _validate_arm(arms.get(name), name) for name in ARM_NAMES},
        "diagnostics": diagnostics,
    }


def encode_pi_packet(packet: dict[str, Any]) -> bytes:
    """Validate and encode one compact Pi sensor datagram."""
    clean = decode_pi_packet(
        json.dumps(packet, allow_nan=False, separators=(",", ":")).encode("utf-8")
    )
    return json.dumps(clean, allow_nan=False, separators=(",", ":")).encode("utf-8")


@dataclass
class PiEventState:
    """Persistent GPIO event counters repeated in every sensor datagram."""

    session_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    counters: dict[str, int] = field(default_factory=lambda: {name: 0 for name in EVENT_NAMES})
    recording_requested: bool = False
    quitting: bool = False

    def handle(self, event: Event) -> None:
        if event is Event.ZERO:
            self.counters["zero"] += 1
            self.recording_requested = False
        elif event in (Event.TOGGLE, Event.START, Event.STOP):
            self.counters["toggle"] += 1
            wants_start = event is Event.START or (
                event is Event.TOGGLE and not self.recording_requested
            )
            if wants_start:
                self.counters["start"] += 1
                self.recording_requested = True
            else:
                self.counters["stop"] += 1
                self.recording_requested = False
        elif event is Event.CANCEL:
            self.counters["cancel"] += 1
            self.recording_requested = False
        elif event is Event.QUIT:
            self.counters["quit"] += 1
            self.recording_requested = False
            self.quitting = True


class PiPacketReceiver:
    """Receive only the newest valid packet from the expected Raspberry Pi."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = 9139,
        *,
        expected_host: str | None = None,
    ) -> None:
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RECEIVE_BUFFER_BYTES)
        self.sock.bind((host, port))
        self.port = self.sock.getsockname()[1]
        self.sock.setblocking(False)
        self.expected_addresses = self._resolve(expected_host)
        self.latest: dict[str, Any] | None = None
        self.received_at: float | None = None
        self.source_address: str | None = None
        self.bad_packets = 0
        self.unexpected_sources = 0

    @staticmethod
    def _resolve(host: str | None) -> set[str] | None:
        if host is None or not host.strip():
            return None
        try:
            return {item[4][0] for item in socket.getaddrinfo(host, None, socket.AF_INET)}
        except socket.gaierror as exc:
            raise ValueError(f"cannot resolve expected Pi host {host!r}: {exc}") from exc

    def poll(self) -> tuple[dict[str, Any] | None, float | None]:
        while True:
            try:
                data, address = self.sock.recvfrom(65_535)
            except BlockingIOError:
                break
            if self.expected_addresses is not None and address[0] not in self.expected_addresses:
                self.unexpected_sources += 1
                continue
            try:
                packet = decode_pi_packet(data)
            except (ValueError, TypeError, UnicodeDecodeError, json.JSONDecodeError):
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
            self.source_address = address[0]
        age = None if self.received_at is None else time.monotonic() - self.received_at
        return self.latest, age

    def close(self) -> None:
        self.sock.close()

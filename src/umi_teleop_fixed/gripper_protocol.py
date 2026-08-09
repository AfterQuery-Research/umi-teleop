from __future__ import annotations

import json
import math
from dataclasses import dataclass

PROTOCOL_VERSION = 3


@dataclass(frozen=True)
class GripperSample:
    width_mm: float | None
    valid: bool
    visible_tag_ids: tuple[int, ...]
    captured_monotonic_ns: int | None
    error: str | None = None

    @classmethod
    def invalid(
        cls,
        *,
        visible_tag_ids: tuple[int, ...] = (),
        captured_monotonic_ns: int | None = None,
        error: str | None = None,
    ) -> GripperSample:
        return cls(
            width_mm=None,
            valid=False,
            visible_tag_ids=visible_tag_ids,
            captured_monotonic_ns=captured_monotonic_ns,
            error=error,
        )


@dataclass(frozen=True)
class ButtonSample:
    pressed: bool
    engage_count: int
    quit_count: int


@dataclass(frozen=True)
class GripperPacket:
    sequence: int
    sender: str
    stream_id: str
    sent_monotonic_ns: int
    left: GripperSample
    right: GripperSample
    button: ButtonSample | None = None


def _sample_payload(sample: GripperSample, *, sent_monotonic_ns: int) -> dict[str, object]:
    if sample.valid:
        if sample.width_mm is None or not math.isfinite(sample.width_mm):
            raise ValueError("a valid gripper sample requires a finite width_mm")
        if sample.width_mm <= 0:
            raise ValueError("width_mm must be positive")
        if sample.captured_monotonic_ns is None:
            raise ValueError("a valid gripper sample requires a capture timestamp")
    elif sample.width_mm is not None:
        raise ValueError("an invalid gripper sample must use width_mm=None")

    age_ms: float | None = None
    if sample.captured_monotonic_ns is not None:
        age_ms = max(0.0, (sent_monotonic_ns - sample.captured_monotonic_ns) / 1e6)

    return {
        "width_mm": sample.width_mm,
        "valid": sample.valid,
        "visible_tag_ids": list(sample.visible_tag_ids),
        "captured_monotonic_ns": sample.captured_monotonic_ns,
        "age_ms": age_ms,
        "error": sample.error,
    }


def _button_payload(sample: ButtonSample | None) -> dict[str, object] | None:
    if sample is None:
        return None
    for name, value in (
        ("engage_count", sample.engage_count),
        ("quit_count", sample.quit_count),
    ):
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError(f"button {name} must be a non-negative integer")
    if not isinstance(sample.pressed, bool):
        raise ValueError("button pressed must be boolean")
    return {
        "pressed": sample.pressed,
        "engage_count": sample.engage_count,
        "quit_count": sample.quit_count,
    }


def encode_packet(packet: GripperPacket) -> bytes:
    if packet.sequence < 0:
        raise ValueError("sequence must be non-negative")
    if not packet.sender:
        raise ValueError("sender must not be empty")
    if not packet.stream_id:
        raise ValueError("stream_id must not be empty")

    payload = {
        "version": PROTOCOL_VERSION,
        "sequence": packet.sequence,
        "sender": packet.sender,
        "stream_id": packet.stream_id,
        "sent_monotonic_ns": packet.sent_monotonic_ns,
        "left": _sample_payload(packet.left, sent_monotonic_ns=packet.sent_monotonic_ns),
        "right": _sample_payload(packet.right, sent_monotonic_ns=packet.sent_monotonic_ns),
        "button": _button_payload(packet.button),
    }
    return json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")


def decode_packet(data: bytes) -> dict[str, object]:
    payload = json.loads(data)
    if not isinstance(payload, dict):
        raise ValueError("gripper packet must be a JSON object")
    if payload.get("version") != PROTOCOL_VERSION:
        raise ValueError(f"unsupported gripper protocol version: {payload.get('version')!r}")
    for key in (
        "sequence",
        "sender",
        "stream_id",
        "sent_monotonic_ns",
        "left",
        "right",
        "button",
    ):
        if key not in payload:
            raise ValueError(f"gripper packet is missing {key!r}")
    return payload

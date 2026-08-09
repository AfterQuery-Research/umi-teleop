from __future__ import annotations

import ipaddress
import math
import socket
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from .gripper_protocol import decode_packet

SIDES = ("left", "right")


@dataclass(frozen=True)
class ReceivedGripperSample:
    width_mm: float
    visible_tag_ids: tuple[int, ...]
    received_at: float
    camera_age_seconds: float
    sequence: int
    stream_id: str
    sender: str
    source_ip: str


@dataclass(frozen=True)
class ReceivedButtonState:
    pressed: bool
    engage_count: int
    quit_count: int


@dataclass(frozen=True)
class ReceivedButtonEvents:
    engage_count: int
    quit_count: int
    pressed: bool
    stream_id: str
    sender: str
    source_ip: str


@dataclass(frozen=True)
class ReceiverStats:
    packets: int
    wrong_source: int
    malformed: int
    out_of_order: int
    invalid_samples: int


class GripperReceiver:
    """Keep the newest validated gripper width from one UDP sender."""

    def __init__(
        self,
        *,
        sender_ip: str,
        port: int = 9130,
        bind_host: str = "0.0.0.0",
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        address = ipaddress.ip_address(sender_ip)
        if address.version != 4:
            raise ValueError("gripper sender_ip must be an IPv4 address")
        if not 1 <= port <= 65535:
            raise ValueError("gripper port must be between 1 and 65535")
        self.sender_ip = str(address)
        self.port = port
        self.bind_host = bind_host
        self._monotonic = monotonic
        self._lock = threading.Lock()
        self._latest: dict[str, ReceivedGripperSample] = {}
        self._stream_id: str | None = None
        self._last_sequence = -1
        self._last_packet_at: float | None = None
        self._button_state: ReceivedButtonState | None = None
        self._button_stream_id: str | None = None
        self._button_sender: str | None = None
        self._button_source_ip: str | None = None
        self._pending_engage = 0
        self._pending_quit = 0
        self._packets = 0
        self._wrong_source = 0
        self._malformed = 0
        self._out_of_order = 0
        self._invalid_samples = 0
        self._socket: socket.socket | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def __enter__(self) -> GripperReceiver:
        if self._socket is not None:
            raise RuntimeError("gripper receiver is already running")
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        udp.bind((self.bind_host, self.port))
        udp.settimeout(0.1)
        self._socket = udp
        self._thread = threading.Thread(target=self._run, name="gripper-udp", daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *_exc_info: object) -> None:
        self.close()

    def close(self) -> None:
        self._stop.set()
        if self._socket is not None:
            self._socket.close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._socket = None
        self._thread = None

    def _run(self) -> None:
        assert self._socket is not None
        while not self._stop.is_set():
            try:
                data, address = self._socket.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                raise
            self.process_datagram(data, source_ip=address[0], received_at=self._monotonic())

    def process_datagram(self, data: bytes, *, source_ip: str, received_at: float) -> bool:
        """Validate and store one datagram; exposed for deterministic tests."""
        if source_ip != self.sender_ip:
            with self._lock:
                self._wrong_source += 1
            return False
        try:
            payload = decode_packet(data)
            sequence = payload["sequence"]
            stream_id = payload["stream_id"]
            sender = payload["sender"]
            if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
                raise ValueError("sequence must be a non-negative integer")
            if not isinstance(stream_id, str) or not stream_id:
                raise ValueError("stream_id must be a nonempty string")
            if not isinstance(sender, str) or not sender:
                raise ValueError("sender must be a nonempty string")
            samples = {
                side: self._decode_sample(
                    payload[side],
                    side=side,
                    sequence=sequence,
                    stream_id=stream_id,
                    sender=sender,
                    source_ip=source_ip,
                    received_at=received_at,
                )
                for side in SIDES
            }
            button = self._decode_button(payload["button"])
        except (KeyError, TypeError, ValueError):
            with self._lock:
                self._malformed += 1
            return False

        with self._lock:
            new_stream = stream_id != self._stream_id
            if new_stream:
                self._stream_id = stream_id
                self._last_sequence = -1
                self._latest.clear()
            if sequence <= self._last_sequence:
                self._out_of_order += 1
                return False
            if not new_stream:
                if (button is None) != (self._button_state is None):
                    self._malformed += 1
                    return False
                if button is not None and self._button_state is not None:
                    if (
                        button.engage_count < self._button_state.engage_count
                        or button.quit_count < self._button_state.quit_count
                    ):
                        self._malformed += 1
                        return False
                    self._pending_engage += (
                        button.engage_count - self._button_state.engage_count
                    )
                    self._pending_quit += button.quit_count - self._button_state.quit_count
            else:
                self._pending_engage = 0
                self._pending_quit = 0
            self._last_sequence = sequence
            self._last_packet_at = received_at
            self._button_state = button
            self._button_stream_id = stream_id
            self._button_sender = sender
            self._button_source_ip = source_ip
            self._packets += 1
            for side, sample in samples.items():
                if sample is None:
                    self._invalid_samples += 1
                else:
                    self._latest[side] = sample
        return True

    @staticmethod
    def _decode_button(value: object) -> ReceivedButtonState | None:
        if value is None:
            return None
        if not isinstance(value, dict):
            raise ValueError("button must be an object or null")
        pressed = value.get("pressed")
        engage_count = value.get("engage_count")
        quit_count = value.get("quit_count")
        if not isinstance(pressed, bool):
            raise ValueError("button pressed must be boolean")
        for name, count in (("engage_count", engage_count), ("quit_count", quit_count)):
            if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                raise ValueError(f"button {name} must be a non-negative integer")
        return ReceivedButtonState(
            pressed=pressed,
            engage_count=engage_count,
            quit_count=quit_count,
        )

    @staticmethod
    def _decode_sample(
        value: object,
        *,
        side: str,
        sequence: int,
        stream_id: str,
        sender: str,
        source_ip: str,
        received_at: float,
    ) -> ReceivedGripperSample | None:
        if not isinstance(value, dict):
            raise ValueError(f"{side} sample must be an object")
        valid = value.get("valid")
        width = value.get("width_mm")
        age_ms = value.get("age_ms")
        visible = value.get("visible_tag_ids")
        if not isinstance(valid, bool):
            raise ValueError(f"{side} valid must be boolean")
        if not isinstance(visible, list) or any(
            isinstance(tag, bool) or not isinstance(tag, int) for tag in visible
        ):
            raise ValueError(f"{side} visible_tag_ids must be integers")
        if not valid:
            if width is not None:
                raise ValueError(f"invalid {side} sample must have null width")
            return None
        if isinstance(width, bool) or not isinstance(width, (int, float)):
            raise ValueError(f"valid {side} sample requires numeric width_mm")
        if not math.isfinite(width) or width <= 0:
            raise ValueError(f"{side} width_mm must be positive and finite")
        if isinstance(age_ms, bool) or not isinstance(age_ms, (int, float)):
            raise ValueError(f"valid {side} sample requires numeric age_ms")
        if not math.isfinite(age_ms) or age_ms < 0:
            raise ValueError(f"{side} age_ms must be non-negative and finite")
        return ReceivedGripperSample(
            width_mm=float(width),
            visible_tag_ids=tuple(visible),
            received_at=received_at,
            camera_age_seconds=float(age_ms) / 1000.0,
            sequence=sequence,
            stream_id=stream_id,
            sender=sender,
            source_ip=source_ip,
        )

    def snapshot(
        self, *, now: float, stale_after_seconds: float
    ) -> dict[str, ReceivedGripperSample]:
        samples, errors = self.read(now=now, stale_after_seconds=stale_after_seconds)
        if errors:
            raise RuntimeError("; ".join(errors[side] for side in SIDES if side in errors))
        return samples

    def poll_button_events(
        self, *, now: float, stale_after_seconds: float
    ) -> ReceivedButtonEvents:
        """Return each cumulative GPIO action exactly once despite UDP packet loss."""
        if not math.isfinite(stale_after_seconds) or stale_after_seconds <= 0:
            raise ValueError("button stale_after_seconds must be positive")
        with self._lock:
            if self._last_packet_at is None:
                raise RuntimeError("no Pi packets received")
            age = now - self._last_packet_at
            if age > stale_after_seconds:
                raise RuntimeError(f"Pi button stream is stale ({age:.3f}s)")
            if self._button_state is None:
                raise RuntimeError("Pi stream does not contain a GPIO button")
            assert self._button_stream_id is not None
            assert self._button_sender is not None
            assert self._button_source_ip is not None
            events = ReceivedButtonEvents(
                engage_count=self._pending_engage,
                quit_count=self._pending_quit,
                pressed=self._button_state.pressed,
                stream_id=self._button_stream_id,
                sender=self._button_sender,
                source_ip=self._button_source_ip,
            )
            self._pending_engage = 0
            self._pending_quit = 0
            return events

    def read(
        self, *, now: float, stale_after_seconds: float
    ) -> tuple[dict[str, ReceivedGripperSample], dict[str, str]]:
        """Return fresh sides independently so one camera cannot freeze the other."""
        if not math.isfinite(stale_after_seconds) or stale_after_seconds <= 0:
            raise ValueError("gripper stale_after_seconds must be positive")
        with self._lock:
            if self._last_packet_at is None:
                return {}, {side: "no gripper packets received" for side in SIDES}
            packet_age = now - self._last_packet_at
            if packet_age > stale_after_seconds:
                message = f"gripper packet stream is stale ({packet_age:.3f}s)"
                return {}, {side: message for side in SIDES}
            result = dict(self._latest)
        errors = {}
        for side in SIDES:
            if side not in result:
                errors[side] = f"no valid {side} gripper width"
        for side, sample in result.items():
            total_age = max(0.0, now - sample.received_at) + sample.camera_age_seconds
            if total_age > stale_after_seconds:
                errors[side] = f"{side} gripper width is stale ({total_age:.3f}s)"
        return {side: sample for side, sample in result.items() if side not in errors}, errors

    def stats(self) -> ReceiverStats:
        with self._lock:
            return ReceiverStats(
                packets=self._packets,
                wrong_source=self._wrong_source,
                malformed=self._malformed,
                out_of_order=self._out_of_order,
                invalid_samples=self._invalid_samples,
            )

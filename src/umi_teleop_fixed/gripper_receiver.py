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
        except (KeyError, TypeError, ValueError):
            with self._lock:
                self._malformed += 1
            return False

        with self._lock:
            if stream_id != self._stream_id:
                self._stream_id = stream_id
                self._last_sequence = -1
                self._latest.clear()
            if sequence <= self._last_sequence:
                self._out_of_order += 1
                return False
            self._last_sequence = sequence
            self._last_packet_at = received_at
            self._packets += 1
            for side, sample in samples.items():
                if sample is None:
                    self._invalid_samples += 1
                else:
                    self._latest[side] = sample
        return True

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

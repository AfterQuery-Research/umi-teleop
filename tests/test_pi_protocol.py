from __future__ import annotations

import socket
import time

import pytest

from umi_teleop.pi_protocol import (
    EVENT_NAMES,
    PiEventState,
    PiPacketReceiver,
    decode_pi_packet,
    encode_pi_packet,
)
from umi_teleop.types import Event


def packet(seq: int = 0) -> dict:
    arm = {
        "width_mm": 80.0,
        "width_age_s": 0.01,
        "tags_visible": True,
        "camera_ok": True,
        "frame_age_s": 0.02,
        "frames_seen": 10,
        "detections": 8,
        "misses": 2,
        "errors": 0,
    }
    return {
        "version": 1,
        "source_id": "pi-test",
        "session_id": "session",
        "seq": seq,
        "uptime_s": 1.0,
        "events": {name: 0 for name in EVENT_NAMES},
        "arms": {"left": dict(arm), "right": dict(arm)},
        "diagnostics": [],
    }


def test_pi_packet_round_trip_and_health_validation() -> None:
    assert decode_pi_packet(encode_pi_packet(packet())) == packet()
    invalid = packet()
    invalid["arms"]["left"]["camera_ok"] = "yes"
    with pytest.raises(ValueError, match="camera_ok"):
        encode_pi_packet(invalid)


def test_pi_event_counters_retain_start_stop_cancel_and_quit() -> None:
    state = PiEventState(session_id="buttons")
    for event in (Event.ZERO, Event.TOGGLE, Event.TOGGLE, Event.START, Event.CANCEL, Event.QUIT):
        state.handle(event)

    assert state.counters == {
        "zero": 1,
        "toggle": 3,
        "start": 2,
        "stop": 1,
        "cancel": 1,
        "quit": 1,
    }
    assert state.quitting and not state.recording_requested


def test_pi_receiver_ignores_old_sequences_and_tracks_arrival_age() -> None:
    receiver = PiPacketReceiver("127.0.0.1", 0, expected_host="127.0.0.1")
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(encode_pi_packet(packet(2)), ("127.0.0.1", receiver.port))
        sender.sendto(encode_pi_packet(packet(1)), ("127.0.0.1", receiver.port))
        deadline = time.monotonic() + 1.0
        received = None
        age = None
        while received is None and time.monotonic() < deadline:
            received, age = receiver.poll()
            time.sleep(0.005)
        assert received is not None and received["seq"] == 2
        assert age is not None and age >= 0.0
        assert receiver.source_address == "127.0.0.1"
    finally:
        sender.close()
        receiver.close()

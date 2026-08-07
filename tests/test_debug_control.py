from __future__ import annotations

import json
import socket
import threading
import time

import pytest

from umi_teleop.debug_control import DebugControlReceiver, send_command
from umi_teleop.types import Event


def _free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])
    finally:
        sock.close()


def test_debug_control_round_trip_returns_applied_source_state() -> None:
    port = _free_udp_port()
    receiver = DebugControlReceiver(port)
    result: list[dict] = []

    def send() -> None:
        result.append(send_command("zero", port=port, timeout_s=1.0))

    thread = threading.Thread(target=send)
    thread.start()
    try:
        deadline = time.monotonic() + 1.0
        requests = []
        while not requests and time.monotonic() < deadline:
            requests = receiver.poll()
            time.sleep(0.001)
        assert len(requests) == 1
        assert requests[0].event is Event.ZERO
        receiver.reply(
            requests[0],
            action="zero pending after 0.4s settle",
            requested_enabled=False,
            zero_id=3,
            toggle_id=5,
            cancel_id=1,
            quit_id=0,
        )
    finally:
        thread.join(timeout=1.0)
        receiver.close()

    assert not thread.is_alive()
    assert result[0]["command"] == "zero"
    assert result[0]["action"] == "zero pending after 0.4s settle"
    assert result[0]["zero_id"] == 3


def test_debug_control_status_has_no_control_event() -> None:
    port = _free_udp_port()
    receiver = DebugControlReceiver(port)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.sendto(
        json.dumps({"version": 1, "request_id": "status-request", "command": "status"}).encode(),
        ("127.0.0.1", port),
    )
    try:
        deadline = time.monotonic() + 1.0
        requests = []
        while not requests and time.monotonic() < deadline:
            requests = receiver.poll()
        assert len(requests) == 1
        assert requests[0].command == "status"
        assert requests[0].event is None
    finally:
        sender.close()
        receiver.close()


def test_debug_control_rejects_non_loopback_destination() -> None:
    with pytest.raises(ValueError, match="loopback"):
        send_command("status", host="192.0.2.1")


@pytest.mark.parametrize("port", [0, 65_536])
def test_debug_control_rejects_invalid_port(port: int) -> None:
    with pytest.raises(ValueError, match="port"):
        send_command("status", port=port)

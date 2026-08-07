"""Opt-in loopback controls for unattended teleoperation diagnostics."""

from __future__ import annotations

import ipaddress
import json
import socket
import sys
import time
import uuid
from dataclasses import dataclass
from typing import Any, Literal, cast

import tyro

from umi_teleop.types import Event

DEBUG_CONTROL_VERSION = 1
MAX_CONTROL_BYTES = 4096
ControlName = Literal["zero", "start", "stop", "cancel", "quit", "status"]
CONTROL_EVENTS: dict[str, Event] = {
    "zero": Event.ZERO,
    "start": Event.START,
    "stop": Event.STOP,
    "cancel": Event.CANCEL,
    "quit": Event.QUIT,
}


@dataclass(frozen=True)
class DebugControlRequest:
    request_id: str
    command: ControlName
    address: tuple[str, int]

    @property
    def event(self) -> Event | None:
        return CONTROL_EVENTS.get(self.command)


def _loopback_address(host: str) -> str:
    """Resolve a host and require every result to be IPv4 loopback."""
    try:
        addresses = {item[4][0] for item in socket.getaddrinfo(host, None, socket.AF_INET)}
    except socket.gaierror as exc:
        raise ValueError(f"cannot resolve debug-control host {host!r}: {exc}") from exc
    if not addresses or any(not ipaddress.ip_address(address).is_loopback for address in addresses):
        raise ValueError("debug control is restricted to the local loopback interface")
    return sorted(addresses)[0]


def _decode_request(data: bytes, address: tuple[str, int]) -> DebugControlRequest:
    if not data or len(data) > MAX_CONTROL_BYTES:
        raise ValueError("debug-control request is empty or too large")
    value = json.loads(data.decode("utf-8"))
    if not isinstance(value, dict) or value.get("version") != DEBUG_CONTROL_VERSION:
        raise ValueError("unsupported debug-control request")
    request_id = value.get("request_id")
    command = value.get("command")
    if not isinstance(request_id, str) or not request_id or len(request_id) > 128:
        raise ValueError("debug-control request_id is invalid")
    if command not in {*CONTROL_EVENTS, "status"}:
        raise ValueError(f"unknown debug-control command {command!r}")
    return DebugControlRequest(request_id, cast(ControlName, command), address)


class DebugControlReceiver:
    """Receive explicit diagnostic commands from another aq-yam process."""

    def __init__(self, port: int, host: str = "127.0.0.1") -> None:
        if not 1 <= port <= 65_535:
            raise ValueError("debug-control port must be between 1 and 65535")
        address = _loopback_address(host)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((address, port))
        self.sock.setblocking(False)
        self.bad_requests = 0
        self.reply_errors = 0

    def poll(self) -> list[DebugControlRequest]:
        requests: list[DebugControlRequest] = []
        while True:
            try:
                data, address = self.sock.recvfrom(MAX_CONTROL_BYTES + 1)
            except BlockingIOError:
                break
            try:
                requests.append(_decode_request(data, address))
            except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError):
                self.bad_requests += 1
        return requests

    def reply(
        self,
        request: DebugControlRequest,
        *,
        action: str,
        requested_enabled: bool,
        zero_id: int,
        toggle_id: int,
        cancel_id: int,
        quit_id: int,
    ) -> None:
        payload = {
            "version": DEBUG_CONTROL_VERSION,
            "request_id": request.request_id,
            "command": request.command,
            "action": action,
            "requested_enabled": requested_enabled,
            "zero_id": zero_id,
            "toggle_id": toggle_id,
            "cancel_id": cancel_id,
            "quit_id": quit_id,
        }
        try:
            self.sock.sendto(
                json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8"),
                request.address,
            )
        except OSError:
            # A diagnostic client disappearing must never take down sensor fusion.
            self.reply_errors += 1

    def close(self) -> None:
        self.sock.close()


def send_command(
    command: ControlName,
    *,
    host: str = "127.0.0.1",
    port: int = 9141,
    timeout_s: float = 2.0,
) -> dict[str, Any]:
    """Send one command and wait for the matching source acknowledgement."""
    if timeout_s <= 0.0:
        raise ValueError("debug-control timeout must be positive")
    if not 1 <= port <= 65_535:
        raise ValueError("debug-control port must be between 1 and 65535")
    if command not in {*CONTROL_EVENTS, "status"}:
        raise ValueError(f"unknown debug-control command {command!r}")
    target = (_loopback_address(host), port)
    request_id = uuid.uuid4().hex
    payload = json.dumps(
        {
            "version": DEBUG_CONTROL_VERSION,
            "request_id": request_id,
            "command": command,
        },
        separators=(",", ":"),
    ).encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    deadline = time.monotonic() + timeout_s
    try:
        sock.sendto(payload, target)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError
            sock.settimeout(remaining)
            data, address = sock.recvfrom(MAX_CONTROL_BYTES + 1)
            if address != target or len(data) > MAX_CONTROL_BYTES:
                continue
            response = json.loads(data.decode("utf-8"))
            if (
                isinstance(response, dict)
                and response.get("version") == DEBUG_CONTROL_VERSION
                and response.get("request_id") == request_id
            ):
                return response
    except TimeoutError as exc:
        raise RuntimeError(
            f"no debug-control response from {target[0]}:{target[1]}; "
            "start umi-teleop with --debug.enable"
        ) from exc
    finally:
        sock.close()


@dataclass(frozen=True)
class Args:
    command: tyro.conf.Positional[ControlName]
    """Command to inject into the running source."""

    port: int = 9141
    timeout_s: float = 2.0


def main() -> None:
    try:
        response = send_command(**vars(tyro.cli(Args)))
        print(json.dumps(response, sort_keys=True))
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"DEBUG CONTROL ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()

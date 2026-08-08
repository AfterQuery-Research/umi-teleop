"""SocketCAN discovery keyed by stable USB adapter serials."""

from __future__ import annotations

import json
import subprocess
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

ARPHRD_CAN = 280
HEALTHY_CAN_STATE = "ERROR-ACTIVE"


@dataclass(frozen=True)
class CANInterfaceInfo:
    """Observed state for one Linux SocketCAN interface."""

    name: str
    adapter_serial: str | None
    driver: str | None
    is_up: bool
    lower_up: bool
    operating_state: str | None
    can_state: str | None
    bitrate: int | None
    is_fd: bool

    @property
    def link_state(self) -> str:
        states = ["UP" if self.is_up else "DOWN"]
        if self.lower_up:
            states.append("LOWER_UP")
        return "/".join(states)


IPDetailsLoader = Callable[[str], dict[str, Any]]


def _read_text(path: Path) -> str | None:
    try:
        value = path.read_text(encoding="utf-8").strip()
    except (FileNotFoundError, OSError):
        return None
    return value or None


def _device_attribute(device_path: Path, attribute: str) -> str | None:
    try:
        current = device_path.resolve(strict=True)
    except (FileNotFoundError, OSError):
        return None

    for parent in (current, *current.parents):
        value = _read_text(parent / attribute)
        if value is not None:
            return value
    return None


def _device_driver(device_path: Path) -> str | None:
    try:
        return (device_path / "driver").resolve(strict=True).name
    except (FileNotFoundError, OSError):
        return None


def load_ip_link_details(interface: str) -> dict[str, Any]:
    """Read one interface through iproute2's JSON output."""
    try:
        result = subprocess.run(
            ["ip", "-details", "-json", "link", "show", "dev", interface],
            check=False,
            capture_output=True,
            text=True,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("The `ip` command is required to inspect SocketCAN") from exc

    if result.returncode != 0:
        message = result.stderr.strip() or f"exit status {result.returncode}"
        raise RuntimeError(f"Could not inspect {interface}: {message}")
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"The `ip` command returned invalid JSON for {interface}") from exc
    if not isinstance(payload, list) or len(payload) != 1 or not isinstance(payload[0], dict):
        raise RuntimeError(f"The `ip` command returned an unexpected result for {interface}")
    return payload[0]


def _parse_link_details(
    details: dict[str, Any],
) -> tuple[bool, bool, str | None, str | None, int | None, bool]:
    flags = details.get("flags", [])
    flags = flags if isinstance(flags, list) else []
    operating_state = details.get("operstate")
    operating_state = operating_state if isinstance(operating_state, str) else None

    link_info = details.get("linkinfo", {})
    info_data = link_info.get("info_data", {}) if isinstance(link_info, dict) else {}
    info_data = info_data if isinstance(info_data, dict) else {}
    can_state = info_data.get("state")
    can_state = can_state if isinstance(can_state, str) else None

    bit_timing = info_data.get("bittiming", {})
    bitrate_value = bit_timing.get("bitrate") if isinstance(bit_timing, dict) else None
    bitrate = int(bitrate_value) if isinstance(bitrate_value, int | float) else None

    control_mode = info_data.get("ctrlmode", [])
    is_fd = isinstance(control_mode, list) and "FD" in control_mode
    return "UP" in flags, "LOWER_UP" in flags, operating_state, can_state, bitrate, is_fd


def discover_can_interfaces(
    sysfs_net_root: Path = Path("/sys/class/net"),
    *,
    ip_details_loader: IPDetailsLoader = load_ip_link_details,
) -> list[CANInterfaceInfo]:
    """Discover every SocketCAN interface and its parent USB serial."""
    if not sysfs_net_root.is_dir():
        return []

    interfaces: list[CANInterfaceInfo] = []
    for interface_path in sorted(sysfs_net_root.iterdir(), key=lambda path: path.name):
        link_type = _read_text(interface_path / "type")
        try:
            is_can = link_type is not None and int(link_type, 0) == ARPHRD_CAN
        except ValueError:
            is_can = False
        if not is_can:
            continue

        parsed = _parse_link_details(ip_details_loader(interface_path.name))
        device_path = interface_path / "device"
        interfaces.append(
            CANInterfaceInfo(
                name=interface_path.name,
                adapter_serial=_device_attribute(device_path, "serial"),
                driver=_device_driver(device_path),
                is_up=parsed[0],
                lower_up=parsed[1],
                operating_state=parsed[2],
                can_state=parsed[3],
                bitrate=parsed[4],
                is_fd=parsed[5],
            )
        )
    return interfaces


def format_can_interfaces(interfaces: list[CANInterfaceInfo]) -> str:
    """Render a compact inventory suitable for setup output."""
    if not interfaces:
        return "No SocketCAN interfaces discovered."

    headings = ("Interface", "USB serial", "Driver", "Link", "CAN state", "Bitrate", "Mode")
    rows = [
        (
            interface.name,
            interface.adapter_serial or "-",
            interface.driver or "-",
            interface.link_state,
            interface.can_state or "-",
            str(interface.bitrate) if interface.bitrate is not None else "unset",
            "CAN FD" if interface.is_fd else "Classic CAN",
        )
        for interface in interfaces
    ]
    widths = [max(len(headings[i]), *(len(row[i]) for row in rows)) for i in range(len(headings))]
    rendered = ["  ".join(value.ljust(widths[i]) for i, value in enumerate(headings))]
    rendered.extend(
        "  ".join(value.ljust(widths[i]) for i, value in enumerate(row)) for row in rows
    )
    return "\n".join(rendered)


def resolve_can_interface(
    adapter_serial: str,
    *,
    interfaces: list[CANInterfaceInfo] | None = None,
) -> CANInterfaceInfo:
    """Resolve exactly one interface by a case-insensitive USB serial."""
    serial = adapter_serial.strip()
    if not serial:
        raise ValueError("CAN adapter serial must not be empty")
    discovered = discover_can_interfaces() if interfaces is None else interfaces
    matches = [
        interface
        for interface in discovered
        if interface.adapter_serial is not None
        and interface.adapter_serial.casefold() == serial.casefold()
    ]
    if len(matches) == 1:
        return matches[0]

    inventory = format_can_interfaces(discovered)
    if not matches:
        raise RuntimeError(f"No CAN adapter with USB serial {serial!r} was found.\n{inventory}")
    names = ", ".join(interface.name for interface in matches)
    raise RuntimeError(f"USB serial {serial!r} matched multiple interfaces: {names}.\n{inventory}")


def readiness_error(interface: CANInterfaceInfo, *, bitrate: int) -> str | None:
    """Return why an interface is unsafe to use, or None when ready."""
    problems: list[str] = []
    if not interface.is_up:
        problems.append("interface is DOWN")
    if not interface.lower_up:
        problems.append("LOWER_UP is missing")
    if interface.operating_state != "UP":
        problems.append(f"operating state is {interface.operating_state or 'unknown'}, expected UP")
    if interface.can_state != HEALTHY_CAN_STATE:
        problems.append(
            f"CAN state is {interface.can_state or 'unknown'}, expected {HEALTHY_CAN_STATE}"
        )
    if interface.bitrate != bitrate:
        actual = "unset" if interface.bitrate is None else str(interface.bitrate)
        problems.append(f"bitrate is {actual}, expected {bitrate}")
    if interface.is_fd:
        problems.append("mode is CAN FD, expected Classic CAN")
    return "; ".join(problems) or None

"""Explicit, non-motion SocketCAN setup for the dual YAM controller."""

from __future__ import annotations

import subprocess
import sys
from dataclasses import dataclass

import tyro

from umi_teleop.can import (
    YAM_CAN_BITRATE,
    can_interface_readiness_error,
    discover_can_interfaces,
    format_can_interfaces,
    resolve_can_interface,
)


@dataclass
class Config:
    """List CAN adapters, or configure selected adapters when serials are supplied."""

    left_adapter_serial: str | None = None
    right_adapter_serial: str | None = None
    bitrate: int = YAM_CAN_BITRATE

    def __post_init__(self) -> None:
        for field_name in ("left_adapter_serial", "right_adapter_serial"):
            value = getattr(self, field_name)
            if value is not None:
                value = value.strip()
                if not value:
                    raise ValueError(f"{field_name} must not be empty")
                setattr(self, field_name, value)
        if (
            self.left_adapter_serial is not None
            and self.right_adapter_serial is not None
            and self.left_adapter_serial.casefold() == self.right_adapter_serial.casefold()
        ):
            raise ValueError("left and right adapter serials must be different")
        if self.bitrate <= 0:
            raise ValueError("bitrate must be positive")

    @property
    def adapter_serials(self) -> tuple[str, ...]:
        return tuple(
            serial
            for serial in (self.left_adapter_serial, self.right_adapter_serial)
            if serial is not None
        )


def setup_interface(interface: str, bitrate: int) -> None:
    commands = [
        ["sudo", "ip", "link", "set", interface, "down"],
        [
            "sudo",
            "ip",
            "link",
            "set",
            interface,
            "type",
            "can",
            "bitrate",
            str(bitrate),
            "fd",
            "off",
        ],
        ["sudo", "ip", "link", "set", interface, "up"],
    ]
    for command in commands:
        result = subprocess.run(command, check=False, capture_output=True, text=True)
        if result.returncode != 0:
            message = result.stderr.strip() or f"exit status {result.returncode}"
            raise RuntimeError(f"failed to configure {interface}: {message}")


def run(config: Config) -> None:
    discovered = discover_can_interfaces()
    if not config.adapter_serials:
        print(format_can_interfaces(discovered))
        return

    selected = [
        resolve_can_interface(serial, interfaces=discovered) for serial in config.adapter_serials
    ]
    if len({interface.name for interface in selected}) != len(selected):
        raise RuntimeError("configured adapter serials resolved to the same interface")

    print(f"Configuring Classic CAN at {config.bitrate} bit/s")
    for interface in selected:
        print(f"Configuring {interface.name} ({interface.adapter_serial})...")
        setup_interface(interface.name, config.bitrate)
        refreshed = discover_can_interfaces()
        configured = next((item for item in refreshed if item.name == interface.name), None)
        if configured is None:
            raise RuntimeError(f"configured interface {interface.name} disappeared")
        readiness_error = can_interface_readiness_error(
            configured,
            bitrate=config.bitrate,
            use_fd=False,
        )
        if readiness_error is not None:
            raise RuntimeError(f"interface {interface.name} failed verification: {readiness_error}")
        print(f"[ok] {interface.name}: UP, {configured.bitrate} bit/s, Classic CAN")
    print("Setup complete. No CAN frames or motor commands were sent.")


def main() -> None:
    try:
        run(tyro.cli(Config))
    except (RuntimeError, ValueError) as exc:
        print(f"CAN SETUP ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()

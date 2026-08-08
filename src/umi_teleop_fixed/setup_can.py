"""Discover and configure the YAM robot's SocketCAN adapters."""

from __future__ import annotations

import os
import subprocess
from collections.abc import Callable
from dataclasses import dataclass

import tyro

from umi_teleop_fixed.can import (
    CANInterfaceInfo,
    discover_can_interfaces,
    format_can_interfaces,
    readiness_error,
    resolve_can_interface,
)

CommandRunner = Callable[..., subprocess.CompletedProcess[str]]


@dataclass(frozen=True)
class Args:
    """List or configure the YAM robot's classic-CAN links.

    With no adapter serials, the command only lists current SocketCAN state.
    Supplying either serial configures that adapter at 1 Mbps and verifies it.
    """

    left_adapter_serial: str | None = None
    """Stable USB serial for the left arm's CAN adapter."""

    right_adapter_serial: str | None = None
    """Stable USB serial for the right arm's CAN adapter."""

    bitrate: int = 1_000_000
    """Classic-CAN bitrate for both YAM arms."""

    def selections(self) -> tuple[tuple[str, str], ...]:
        selected = []
        for side, value in (
            ("left", self.left_adapter_serial),
            ("right", self.right_adapter_serial),
        ):
            if value is not None:
                serial = value.strip()
                if not serial:
                    raise ValueError(f"{side}_adapter_serial must not be empty")
                selected.append((side, serial))
        if len(selected) == 2 and selected[0][1].casefold() == selected[1][1].casefold():
            raise ValueError("left and right adapter serials must be different")
        if self.bitrate <= 0:
            raise ValueError("bitrate must be positive")
        return tuple(selected)


def configure_interface(
    interface: str,
    *,
    bitrate: int,
    runner: CommandRunner = subprocess.run,
    use_sudo: bool | None = None,
) -> None:
    """Bring one interface down, configure classic CAN, and bring it up."""
    if use_sudo is None:
        use_sudo = os.geteuid() != 0
    prefix = ["sudo"] if use_sudo else []
    commands = [
        [*prefix, "ip", "link", "set", interface, "down"],
        [
            *prefix,
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
        [*prefix, "ip", "link", "set", interface, "up"],
    ]

    for index, command in enumerate(commands):
        result = runner(command, check=False, capture_output=True, text=True)
        if result.returncode == 0:
            continue
        if index == 1:
            runner(commands[2], check=False, capture_output=True, text=True)
        message = result.stderr.strip() or f"exit status {result.returncode}"
        raise RuntimeError(f"Command failed: {' '.join(command)}\n{message}")


def authorize_link_changes(runner: CommandRunner = subprocess.run) -> None:
    """Authorize all privileged link changes before touching either adapter."""
    if os.geteuid() == 0:
        return
    result = runner(["sudo", "-v"], check=False, capture_output=True, text=True)
    if result.returncode == 0:
        return
    message = result.stderr.strip() or f"exit status {result.returncode}"
    raise RuntimeError(
        "sudo authorization is required before changing CAN links. "
        "Run this command in an interactive aq-yam shell.\n"
        f"{message}"
    )


def _resolve_selections(
    selections: tuple[tuple[str, str], ...], interfaces: list[CANInterfaceInfo]
) -> tuple[tuple[str, str, str], ...]:
    resolved = tuple(
        (side, serial, resolve_can_interface(serial, interfaces=interfaces).name)
        for side, serial in selections
    )
    names = [name for _, _, name in resolved]
    if len(names) != len(set(names)):
        raise RuntimeError("Configured serials resolved to the same CAN interface")
    return resolved


def run(args: Args) -> None:
    """Execute the read-only inventory or setup-and-verify workflow."""
    selections = args.selections()
    discovered = discover_can_interfaces()
    print(format_can_interfaces(discovered))

    if not selections:
        print("\nRead-only discovery: no link configuration or CAN frames were sent.")
        return

    resolved = _resolve_selections(selections, discovered)
    needs_configuration = []
    for side, serial, interface_name in resolved:
        interface = resolve_can_interface(serial, interfaces=discovered)
        if readiness_error(interface, bitrate=args.bitrate) is not None:
            needs_configuration.append((side, serial, interface_name))

    if needs_configuration:
        authorize_link_changes()
        for side, serial, interface in needs_configuration:
            print(f"\nConfiguring {side}: {serial} -> {interface}")
            configure_interface(interface, bitrate=args.bitrate)
        verified = discover_can_interfaces()
    else:
        print("\nSelected CAN links are already configured; leaving them up.")
        verified = discovered
    failures = []
    for side, serial, _ in resolved:
        interface = resolve_can_interface(serial, interfaces=verified)
        error = readiness_error(interface, bitrate=args.bitrate)
        if error is not None:
            failures.append(f"{side} ({interface.name}): {error}")

    print("\nVerified SocketCAN state:")
    print(format_can_interfaces(verified))
    if failures:
        raise RuntimeError("CAN verification failed:\n" + "\n".join(failures))

    print("\nCAN setup complete. No CAN frames or motor commands were sent.")


def main() -> None:
    """CLI entry point."""
    try:
        run(tyro.cli(Args))
    except (RuntimeError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error


if __name__ == "__main__":
    main()

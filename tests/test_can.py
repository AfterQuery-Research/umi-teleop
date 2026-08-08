from pathlib import Path

import pytest

from umi_teleop_fixed.can import (
    CANInterfaceInfo,
    discover_can_interfaces,
    format_can_interfaces,
    readiness_error,
    resolve_can_interface,
)


def _add_interface(root: Path, name: str, serial: str, *, link_type: int = 280) -> None:
    interface = root / "net" / name
    interface.mkdir(parents=True)
    (interface / "type").write_text(str(link_type))

    adapter = root / "devices" / f"usb-{name}"
    usb_interface = adapter / f"{name}:1.0"
    usb_interface.mkdir(parents=True)
    (adapter / "serial").write_text(serial)
    driver = root / "drivers" / "gs_usb"
    driver.mkdir(parents=True, exist_ok=True)
    (usb_interface / "driver").symlink_to(driver, target_is_directory=True)
    (interface / "device").symlink_to(usb_interface, target_is_directory=True)


def _details(*, up: bool, bitrate: int | None, fd: bool = False) -> dict:
    info_data: dict = {"state": "ERROR-ACTIVE" if up else "STOPPED"}
    if bitrate is not None:
        info_data["bittiming"] = {"bitrate": bitrate}
    if fd:
        info_data["ctrlmode"] = ["FD"]
    return {
        "flags": ["UP", "LOWER_UP", "NOARP"] if up else ["NOARP"],
        "operstate": "UP" if up else "DOWN",
        "linkinfo": {"info_data": info_data},
    }


def _interface(name: str = "can0", serial: str = "SERIAL", *, ready: bool = True):
    return CANInterfaceInfo(
        name=name,
        adapter_serial=serial,
        driver="gs_usb",
        is_up=ready,
        lower_up=ready,
        operating_state="UP" if ready else "DOWN",
        can_state="ERROR-ACTIVE" if ready else "STOPPED",
        bitrate=1_000_000 if ready else None,
        is_fd=False,
    )


def test_discovery_resolves_usb_serial_instead_of_interface_order(tmp_path):
    _add_interface(tmp_path, "can1", "LEFT-SERIAL")
    _add_interface(tmp_path, "can0", "RIGHT-SERIAL")
    _add_interface(tmp_path, "eth0", "NOT-CAN", link_type=1)

    interfaces = discover_can_interfaces(
        tmp_path / "net",
        ip_details_loader=lambda name: _details(
            up=name == "can1", bitrate=1_000_000 if name == "can1" else None
        ),
    )

    assert [interface.name for interface in interfaces] == ["can0", "can1"]
    assert resolve_can_interface("left-serial", interfaces=interfaces).name == "can1"
    assert resolve_can_interface("RIGHT-SERIAL", interfaces=interfaces).name == "can0"
    assert all(interface.driver == "gs_usb" for interface in interfaces)
    assert "ERROR-ACTIVE" in format_can_interfaces(interfaces)


def test_resolution_error_includes_inventory():
    interfaces = [_interface(serial="KNOWN")]

    with pytest.raises(RuntimeError, match="KNOWN"):
        resolve_can_interface("MISSING", interfaces=interfaces)


def test_readiness_requires_complete_healthy_classic_can_state():
    assert readiness_error(_interface(), bitrate=1_000_000) is None

    wrong = CANInterfaceInfo(
        "can0", "SERIAL", "gs_usb", False, False, "DOWN", "BUS-OFF", 500_000, True
    )
    message = readiness_error(wrong, bitrate=1_000_000)
    assert message is not None
    assert "DOWN" in message
    assert "LOWER_UP" in message
    assert "BUS-OFF" in message
    assert "500000" in message
    assert "Classic CAN" in message

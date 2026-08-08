from types import SimpleNamespace

import pytest

from umi_teleop_fixed import setup_can
from umi_teleop_fixed.can import CANInterfaceInfo


def _interface(name: str, serial: str, *, ready: bool = True) -> CANInterfaceInfo:
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


def test_no_arguments_is_read_only(monkeypatch, capsys):
    monkeypatch.setattr(setup_can, "discover_can_interfaces", lambda: [])
    setup_can.run(setup_can.Args())
    assert "Read-only discovery" in capsys.readouterr().out


def test_args_reject_duplicate_serials():
    with pytest.raises(ValueError, match="must be different"):
        setup_can.Args(left_adapter_serial="SAME", right_adapter_serial="same").selections()


def test_serials_resolve_independently_of_interface_order(monkeypatch, capsys):
    discovered = [_interface("can0", "RIGHT", ready=False), _interface("can1", "LEFT", ready=False)]
    verified = [_interface("can0", "RIGHT"), _interface("can1", "LEFT")]
    configured = []
    inventories = iter((discovered, verified))
    monkeypatch.setattr(setup_can, "discover_can_interfaces", lambda: next(inventories))
    monkeypatch.setattr(setup_can, "authorize_link_changes", lambda: None)
    monkeypatch.setattr(
        setup_can,
        "configure_interface",
        lambda interface, *, bitrate: configured.append((interface, bitrate)),
    )

    setup_can.run(setup_can.Args(left_adapter_serial="LEFT", right_adapter_serial="RIGHT"))

    assert configured == [("can1", 1_000_000), ("can0", 1_000_000)]
    assert "No CAN frames or motor commands were sent" in capsys.readouterr().out


def test_ready_links_are_not_reconfigured(monkeypatch, capsys):
    discovered = [_interface("can0", "LEFT"), _interface("can1", "RIGHT")]
    monkeypatch.setattr(setup_can, "discover_can_interfaces", lambda: discovered)
    monkeypatch.setattr(
        setup_can,
        "authorize_link_changes",
        lambda: pytest.fail("ready links must not request sudo"),
    )
    monkeypatch.setattr(
        setup_can,
        "configure_interface",
        lambda *_args, **_kwargs: pytest.fail("ready links must not be reconfigured"),
    )

    setup_can.run(setup_can.Args(left_adapter_serial="LEFT", right_adapter_serial="RIGHT"))

    assert "already configured" in capsys.readouterr().out


def test_authorization_is_checked_before_link_changes():
    commands = []

    def fake_run(command, **_kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=1, stderr="password required")

    with pytest.raises(RuntimeError, match="interactive aq-yam shell"):
        setup_can.authorize_link_changes(runner=fake_run)

    assert commands == [["sudo", "-v"]]


def test_configure_interface_uses_classic_can_commands():
    commands = []

    def fake_run(command, **_kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0, stderr="")

    setup_can.configure_interface("can7", bitrate=1_000_000, runner=fake_run, use_sudo=False)

    assert commands == [
        ["ip", "link", "set", "can7", "down"],
        ["ip", "link", "set", "can7", "type", "can", "bitrate", "1000000", "fd", "off"],
        ["ip", "link", "set", "can7", "up"],
    ]

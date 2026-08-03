from types import SimpleNamespace

import pytest

from umi_teleop import setup_can as setup_can_module
from umi_teleop.can import CANInterfaceInfo


def interface(name: str, serial: str, *, ready: bool = True) -> CANInterfaceInfo:
    return CANInterfaceInfo(
        name=name,
        adapter_serial=serial,
        driver="gs_usb",
        is_up=ready,
        bitrate=1_000_000 if ready else None,
        is_fd=False,
    )


def test_no_serials_defaults_to_read_only_listing(monkeypatch, capsys):
    monkeypatch.setattr(
        setup_can_module,
        "discover_can_interfaces",
        lambda: [interface("can0", "SERIAL")],
    )

    setup_can_module.run(setup_can_module.Config())

    output = capsys.readouterr().out
    assert "SERIAL" in output
    assert "can0" in output


def test_config_rejects_duplicate_serials():
    with pytest.raises(ValueError, match="must be different"):
        setup_can_module.Config(left_adapter_serial="SAME", right_adapter_serial="same")


def test_setup_resolves_reordered_serials_and_only_configures_links(monkeypatch, capsys):
    config = setup_can_module.Config(left_adapter_serial="LEFT", right_adapter_serial="RIGHT")
    discovered = [interface("can0", "RIGHT"), interface("can1", "LEFT")]
    configured = []
    monkeypatch.setattr(setup_can_module, "discover_can_interfaces", lambda: discovered)
    monkeypatch.setattr(
        setup_can_module,
        "setup_interface",
        lambda name, bitrate: configured.append((name, bitrate)),
    )

    setup_can_module.run(config)

    assert configured == [("can1", 1_000_000), ("can0", 1_000_000)]
    assert "No CAN frames or motor commands were sent" in capsys.readouterr().out


def test_setup_explicitly_disables_can_fd(monkeypatch):
    commands = []

    def fake_run(command, **_kwargs):
        commands.append(command)
        return SimpleNamespace(returncode=0, stderr="")

    monkeypatch.setattr(setup_can_module.subprocess, "run", fake_run)

    setup_can_module.setup_interface("can7", 1_000_000)

    assert commands == [
        ["sudo", "ip", "link", "set", "can7", "down"],
        [
            "sudo",
            "ip",
            "link",
            "set",
            "can7",
            "type",
            "can",
            "bitrate",
            "1000000",
            "fd",
            "off",
        ],
        ["sudo", "ip", "link", "set", "can7", "up"],
    ]

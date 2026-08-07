from __future__ import annotations

from pathlib import Path

import pytest

from umi_teleop import local


def test_physical_local_launcher_is_arm_only_and_has_no_dataset_flags(monkeypatch):
    monkeypatch.setattr(local.sys, "platform", "linux")
    commands = local.build_commands(local.Config(confirm_clear=True), Path("/venv/bin"))

    assert commands["tracking"][0] == "/venv/bin/umi-vive-linux"
    assert "--arm-only" in commands["source"]
    assert "--buttons" in commands["source"]
    assert "keyboard" in commands["source"]
    assert "--arm-only" in commands["controller"]
    assert "--quiet" in commands["source"]
    assert "--quiet" in commands["controller"]
    assert "--confirm-clear" in commands["controller"]
    assert local.DEFAULT_LEFT_CAN in commands["controller"]
    assert local.DEFAULT_RIGHT_CAN in commands["controller"]
    assert not any("dataset" in value for command in commands.values() for value in command)


def test_launcher_finds_sibling_virtual_environment_commands(monkeypatch):
    monkeypatch.setattr(local.sys, "platform", "linux")
    monkeypatch.setattr(local.sys, "executable", "/repo/.venv/bin/python")

    commands = local.build_commands(local.Config(confirm_clear=True))

    assert commands["tracking"][0] == "/repo/.venv/bin/umi-vive-linux"
    assert commands["source"][0] == "/repo/.venv/bin/umi-source"
    assert commands["controller"][0] == "/repo/.venv/bin/umi-yam"


def test_physical_launcher_requires_clear_workspace_confirmation(monkeypatch):
    monkeypatch.setattr(local.sys, "platform", "linux")
    with pytest.raises(ValueError, match="confirm-clear"):
        local.build_commands(local.Config(), Path("/venv/bin"))


def test_simulated_launcher_never_passes_can_or_clear_confirmation(monkeypatch):
    monkeypatch.setattr(local.sys, "platform", "linux")
    controller = local.build_commands(local.Config(simulate=True), Path("/venv/bin"))["controller"]

    assert "--simulate" in controller
    assert "--confirm-clear" not in controller
    assert not any("adapter-serial" in value for value in controller)

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest
import tyro

from umi_teleop import local


def test_tyro_parses_nested_runtime_hardware_and_control_arguments(monkeypatch):
    monkeypatch.setattr(local.sys, "platform", "linux")
    args = tyro.cli(
        local.Args,
        args=[
            "--tracking.left-id",
            "LEFT",
            "--tracking.right-id",
            "RIGHT",
            "--pi.source-host",
            "10.0.0.3",
            "--control.position-scale",
            "2.4",
            "--simulate",
        ],
    )

    assert args.tracking.left_id == "LEFT"
    assert args.pi.source_host == "10.0.0.3"
    assert args.control.position_scale == 2.4


def test_physical_local_launcher_uses_pi_grippers_and_gpio(monkeypatch):
    monkeypatch.setattr(local.sys, "platform", "linux")
    commands = local.build_commands(local.Config(confirm_clear=True), Path("/venv/bin"))

    assert commands["tracking"][0] == "/venv/bin/umi-vive-linux"
    assert "--buttons" in commands["source"]
    assert "pi" in commands["source"]
    assert "--pi-expected-host" in commands["source"]
    assert "--arm-only" not in commands["controller"]
    assert "--quiet" in commands["source"]
    assert "--quiet" in commands["controller"]
    assert commands["controller"][commands["controller"].index("--position-scale") + 1] == "2"
    assert commands["tracking"][commands["tracking"].index("--rate-hz") + 1] == "120"
    assert commands["source"][commands["source"].index("--send-hz") + 1] == "120"
    assert commands["controller"][commands["controller"].index("--max-joint-speed") + 1] == "1"
    assert commands["controller"][commands["controller"].index("--hz") + 1] == "100"
    assert "--synchronous-ik" in commands["controller"]
    assert "--confirm-clear" in commands["controller"]
    assert local.DEFAULT_LEFT_CAN in commands["controller"]
    assert local.DEFAULT_RIGHT_CAN in commands["controller"]
    assert "--rotation-scale" in commands["controller"]
    assert "--max-joint-acceleration" in commands["controller"]
    assert not any(value.startswith("--dataset") for value in commands["controller"])


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


@pytest.mark.parametrize(
    ("group", "field", "value", "message"),
    [
        ("control", "position_scale", 3.1, "position scale"),
        ("control", "max_joint_speed", 2.1, "maximum joint speed"),
        ("tracking", "rate_hz", 241.0, "tracking rate"),
        ("tracking", "fusion_hz", 241.0, "fusion rate"),
        ("control", "controller_hz", 121.0, "controller rate"),
    ],
)
def test_launcher_rejects_unsafe_performance_values(monkeypatch, group, field, value, message):
    monkeypatch.setattr(local.sys, "platform", "linux")
    config = local.Args(confirm_clear=True)
    config = replace(config, **{group: replace(getattr(config, group), **{field: value})})

    with pytest.raises(ValueError, match=message):
        local.build_commands(config, Path("/venv/bin"))

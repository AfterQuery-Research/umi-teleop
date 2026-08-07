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
    assert commands["tracking"][commands["tracking"].index("--max-linear-speed-m-s") + 1] == "8"
    assert (
        commands["tracking"][commands["tracking"].index("--max-angular-speed-deg-s") + 1] == "1440"
    )
    assert commands["tracking"][commands["tracking"].index("--reacquire-ms") + 1] == "250"
    assert commands["source"][commands["source"].index("--send-hz") + 1] == "120"
    assert commands["controller"][commands["controller"].index("--max-joint-speed") + 1] == "1"
    assert commands["controller"][commands["controller"].index("--hz") + 1] == "100"
    assert "--synchronous-ik" in commands["controller"]
    assert "--confirm-clear" in commands["controller"]
    assert local.DEFAULT_LEFT_CAN in commands["controller"]
    assert local.DEFAULT_RIGHT_CAN in commands["controller"]
    assert "--rotation-scale" in commands["controller"]
    assert "--max-joint-acceleration" in commands["controller"]
    assert (
        commands["controller"][commands["controller"].index("--max-zero-translation") + 1] == "0.75"
    )
    assert (
        commands["controller"][commands["controller"].index("--max-zero-rotation-deg") + 1] == "150"
    )
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
    simulated_tracking = replace(local.TrackingArgs(), simulate=True)
    tracking = local.build_commands(
        local.Config(simulate=True, tracking=simulated_tracking), Path("/venv/bin")
    )["tracking"]
    assert "--max-linear-speed-m-s" not in tracking
    assert "--max-angular-speed-deg-s" not in tracking
    assert "--reacquire-ms" not in tracking


@pytest.mark.parametrize(
    ("group", "field", "value", "message"),
    [
        ("control", "position_scale", 3.1, "position scale"),
        ("control", "max_joint_speed", 2.1, "maximum joint speed"),
        ("tracking", "rate_hz", 241.0, "tracking rate"),
        ("tracking", "fusion_hz", 241.0, "fusion rate"),
        ("control", "controller_hz", 121.0, "controller rate"),
        ("tracking", "max_linear_speed_m_s", 100.1, "tracking linear speed"),
        ("tracking", "reacquire_ms", 5.0, "tracking reacquisition"),
        ("control", "max_zero_translation_m", 2.1, "ZERO residual translation"),
    ],
)
def test_launcher_rejects_unsafe_performance_values(monkeypatch, group, field, value, message):
    monkeypatch.setattr(local.sys, "platform", "linux")
    config = local.Args(confirm_clear=True)
    config = replace(config, **{group: replace(getattr(config, group), **{field: value})})

    with pytest.raises(ValueError, match=message):
        local.build_commands(config, Path("/venv/bin"))


class PollSequence:
    def __init__(self, *statuses):
        self.statuses = list(statuses)

    def poll(self):
        if len(self.statuses) > 1:
            return self.statuses.pop(0)
        return self.statuses[0]


class FakeProcess:
    def __init__(self, status):
        self.status = status

    def poll(self):
        return self.status

    def send_signal(self, _signal):
        self.status = 0

    def terminate(self):
        self.status = 0

    def kill(self):
        self.status = 0

    def wait(self, timeout=None):
        del timeout
        return self.status


def test_coordinated_gpio_shutdown_accepts_clean_source_and_controller_exit(monkeypatch):
    monkeypatch.setattr(local.time, "sleep", lambda _seconds: None)
    processes = [
        ("tracking", PollSequence(None)),
        ("source", PollSequence(0)),
        ("controller", PollSequence(None, 0)),
    ]

    assert local.coordinated_shutdown_status(processes) == 0


def test_launcher_returns_success_for_coordinated_gpio_quit(monkeypatch):
    commands = {
        "tracking": [local.sys.executable, "tracking"],
        "source": [local.sys.executable, "source"],
        "controller": [local.sys.executable, "controller"],
    }
    processes = {
        "tracking": FakeProcess(None),
        "source": FakeProcess(0),
        "controller": FakeProcess(0),
    }
    monkeypatch.setattr(local, "build_commands", lambda _args: commands)

    result = local.run(local.Config(simulate=True), popen=lambda command: processes[command[1]])

    assert result == 0
    assert processes["tracking"].poll() == 0


def test_coordinated_gpio_shutdown_propagates_child_failure():
    processes = [
        ("tracking", PollSequence(None)),
        ("source", PollSequence(0)),
        ("controller", PollSequence(3)),
    ]

    assert local.coordinated_shutdown_status(processes) == 3


def test_single_clean_child_exit_is_still_unexpected():
    processes = [
        ("tracking", PollSequence(None)),
        ("source", PollSequence(0)),
        ("controller", PollSequence(None)),
    ]

    assert local.coordinated_shutdown_status(processes, timeout_s=0.0) is None

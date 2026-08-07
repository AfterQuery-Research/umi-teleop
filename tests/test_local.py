from __future__ import annotations

import json
import socket
import threading
import time
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
            "--tracking.startup-attempts",
            "4",
            "--simulate",
        ],
    )

    assert args.tracking.left_id == "LEFT"
    assert args.pi.source_host == "10.0.0.3"
    assert args.control.position_scale == 2.4
    assert args.tracking.startup_attempts == 4


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
    assert commands["tracking"][commands["tracking"].index("--poser") + 1] == "KalmanOnly"
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
        ("tracking", "startup_timeout_s", 1.0, "tracking startup timeout"),
        ("tracking", "startup_attempts", 0, "tracking startup attempts"),
        ("tracking", "poser", "Unsupported", "tracking poser"),
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
    monkeypatch.setattr(
        local,
        "start_tracking_process",
        lambda _args, _command, *, popen: processes["tracking"],
    )

    result = local.run(local.Config(simulate=True), popen=lambda command: processes[command[1]])

    assert result == 0
    assert processes["tracking"].poll() == 0


def test_tracking_startup_probe_requires_both_serials_to_remain_valid():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    target = receiver.getsockname()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    process = FakeProcess(None)

    def send_packets():
        started = time.monotonic()
        sequence = 0
        while time.monotonic() - started < 0.15:
            both_valid = time.monotonic() - started >= 0.03
            packet = {
                "seq": sequence,
                "frame": "libsurvive_tracking",
                "trackers": [
                    {
                        "serial": "LEFT",
                        "valid": True,
                        "continuity_id": 1,
                        "pos": [0.0, 0.0, 0.0],
                        "quat": [1.0, 0.0, 0.0, 0.0],
                    },
                    {
                        "serial": "RIGHT",
                        "valid": both_valid,
                        "continuity_id": 1,
                        "pos": [0.2, 0.0, 0.0],
                        "quat": [1.0, 0.0, 0.0, 0.0],
                    },
                ],
            }
            sender.sendto(json.dumps(packet).encode(), target)
            sequence += 1
            time.sleep(0.005)

    thread = threading.Thread(target=send_packets)
    thread.start()
    try:
        local.wait_for_tracking_ready(
            receiver,
            process,
            expected_serials={"LEFT", "RIGHT"},
            timeout_s=0.5,
            stable_s=0.04,
        )
    finally:
        thread.join()
        sender.close()
        receiver.close()


def test_tracking_startup_probe_rejects_single_tracker_stream():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.sendto(
        json.dumps(
            {
                "seq": 0,
                "frame": "libsurvive_tracking",
                "trackers": [
                    {
                        "serial": "LEFT",
                        "valid": True,
                        "continuity_id": 0,
                        "pos": [0.0, 0.0, 0.0],
                        "quat": [1.0, 0.0, 0.0, 0.0],
                    }
                ],
            }
        ).encode(),
        receiver.getsockname(),
    )
    try:
        with pytest.raises(RuntimeError, match="did not remain valid") as exc_info:
            local.wait_for_tracking_ready(
                receiver,
                FakeProcess(None),
                expected_serials={"LEFT", "RIGHT"},
                timeout_s=0.05,
                stable_s=0.01,
            )
        assert "missing or invalid: RIGHT" in str(exc_info.value)
    finally:
        sender.close()
        receiver.close()


@pytest.mark.parametrize(
    "packet",
    [
        {
            "seq": 0,
            "frame": "libsurvive_tracking",
            "trackers": [
                {"serial": "LEFT", "valid": True, "continuity_id": 0},
                {"serial": "RIGHT", "valid": True, "continuity_id": 0},
            ],
        },
        {
            "seq": 0,
            "frame": "openvr",
            "trackers": [],
        },
    ],
)
def test_tracking_startup_probe_rejects_packets_without_native_valid_poses(packet):
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sender.sendto(json.dumps(packet).encode(), receiver.getsockname())
    try:
        with pytest.raises(RuntimeError, match="did not remain valid"):
            local.wait_for_tracking_ready(
                receiver,
                FakeProcess(None),
                expected_serials={"LEFT", "RIGHT"},
                timeout_s=0.05,
                stable_s=0.01,
            )
    finally:
        sender.close()
        receiver.close()


def test_tracking_startup_probe_requires_advancing_packet_sequence():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packet = {
        "seq": 7,
        "frame": "libsurvive_tracking",
        "trackers": [
            {
                "serial": serial,
                "valid": True,
                "continuity_id": 0,
                "pos": [0.0, 0.0, 0.0],
                "quat": [1.0, 0.0, 0.0, 0.0],
            }
            for serial in ("LEFT", "RIGHT")
        ],
    }

    def repeat_packet():
        for _attempt in range(20):
            sender.sendto(json.dumps(packet).encode(), receiver.getsockname())
            time.sleep(0.003)

    thread = threading.Thread(target=repeat_packet)
    thread.start()
    try:
        with pytest.raises(RuntimeError, match="did not remain valid"):
            local.wait_for_tracking_ready(
                receiver,
                FakeProcess(None),
                expected_serials={"LEFT", "RIGHT"},
                timeout_s=0.1,
                stable_s=0.02,
            )
    finally:
        thread.join()
        sender.close()
        receiver.close()


def test_tracking_process_retries_only_before_controller_start(monkeypatch):
    attempts = []
    processes = []

    def fake_popen(command):
        attempts.append(command)
        process = FakeProcess(None)
        processes.append(process)
        return process

    def readiness(_receiver, _process, **_kwargs):
        if len(attempts) == 1:
            raise RuntimeError("one tracker missing")

    monkeypatch.setattr(local, "wait_for_tracking_ready", readiness)
    monkeypatch.setattr(local.time, "sleep", lambda _seconds: None)
    tracking = replace(
        local.TrackingArgs(),
        port=0,
        startup_attempts=2,
    )

    result = local.start_tracking_process(
        local.Args(tracking=tracking, simulate=True),
        [local.sys.executable, "tracking"],
        popen=fake_popen,
    )

    assert result is processes[1]
    assert processes[0].poll() == 0
    assert processes[1].poll() is None


def test_tracking_startup_refuses_an_occupied_receiver_port():
    occupied = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    occupied.bind(("127.0.0.1", 0))
    tracking = replace(
        local.TrackingArgs(),
        bind="127.0.0.1",
        port=occupied.getsockname()[1],
    )
    try:
        with pytest.raises(RuntimeError, match="cannot bind tracking startup probe"):
            local.start_tracking_process(
                local.Args(tracking=tracking, simulate=True),
                [local.sys.executable, "tracking"],
                popen=lambda _command: pytest.fail("tracking must not start without the port"),
            )
    finally:
        occupied.close()


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

from __future__ import annotations

import socket
from dataclasses import replace
from pathlib import Path

import pytest
import tyro

from umi_teleop import pi_source
from umi_teleop.pi_protocol import PiEventState, decode_pi_packet, encode_pi_packet
from umi_teleop.types import Event


def test_tyro_parses_nested_pi_camera_and_network_arguments() -> None:
    args = tyro.cli(
        pi_source.Args,
        args=[
            "--network.target-host",
            "10.0.0.2",
            "--left.tag-ids",
            "10",
            "11",
            "--right.tag-ids",
            "12",
            "13",
            "--simulate",
        ],
    )

    assert args.network.target_host == "10.0.0.2"
    assert args.left.tag_ids == (10, 11)
    assert args.right.tag_ids == (12, 13)
    assert args.simulate


def test_pi_validation_reports_duplicate_tags_together() -> None:
    args = pi_source.Args(simulate=True)
    args = replace(args, right=replace(args.right, tag_ids=(2, 4)))
    with pytest.raises(ValueError, match="tag IDs must be distinct"):
        pi_source.validate_args(args)


def test_simulated_pi_packet_contains_camera_health_and_event_counters() -> None:
    args = pi_source.Args(simulate=True)
    workers = pi_source._workers(args)
    snapshots = {
        name: worker.snapshot(5.0, camera_timeout_s=1.0) for name, worker in workers.items()
    }
    packet = pi_source._packet(args, PiEventState(session_id="s"), "pi", 3, 4.0, snapshots)
    decoded = decode_pi_packet(encode_pi_packet(packet))

    assert decoded["seq"] == 3
    assert all(decoded["arms"][name]["camera_ok"] for name in ("left", "right"))
    assert not decoded["diagnostics"]


@pytest.mark.integration
def test_pi_runtime_publishes_mocked_gpio_and_camera_state_over_udp(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(1.0)
    port = receiver.getsockname()[1]

    class Worker(pi_source._SimulatedWorker):
        pass

    class Buttons:
        def __init__(self) -> None:
            self.sent = False
            self.closed = False

        def poll(self):
            if not self.sent:
                self.sent = True
                return Event.ZERO
            return None

        def close(self) -> None:
            self.closed = True

    buttons = Buttons()
    monkeypatch.setattr(
        pi_source,
        "_workers",
        lambda _args: {"left": Worker(80.0), "right": Worker(82.0)},
    )
    monkeypatch.setattr(pi_source, "GpioButtons", lambda **_kwargs: buttons)
    left_device = tmp_path / "left-video"
    right_device = tmp_path / "right-video"
    left_calibration = tmp_path / "left-calibration.npz"
    right_calibration = tmp_path / "right-calibration.npz"
    for path in (left_device, right_device, left_calibration, right_calibration):
        path.touch()
    args = pi_source.Args(
        network=replace(pi_source.NetworkArgs(), target_host="127.0.0.1", target_port=port),
        left=replace(
            pi_source._left_camera(),
            device=str(left_device),
            calibration=left_calibration,
        ),
        right=replace(
            pi_source._right_camera(),
            device=str(right_device),
            calibration=right_calibration,
        ),
        max_packets=2,
    )
    try:
        pi_source.run(args)
        packets = [decode_pi_packet(receiver.recv(65_535)) for _ in range(2)]
    finally:
        receiver.close()

    assert [packet["seq"] for packet in packets] == [0, 1]
    assert all(packet["events"]["zero"] == 1 for packet in packets)
    assert all(packet["arms"]["left"]["camera_ok"] for packet in packets)
    assert buttons.closed

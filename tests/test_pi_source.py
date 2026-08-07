from __future__ import annotations

from dataclasses import replace

import pytest
import tyro

from umi_teleop import pi_source
from umi_teleop.pi_protocol import PiEventState, decode_pi_packet, encode_pi_packet


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

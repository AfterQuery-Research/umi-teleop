import pytest

from umi_teleop_fixed.gripper_protocol import (
    ButtonSample,
    GripperPacket,
    GripperSample,
    encode_packet,
)
from umi_teleop_fixed.gripper_receiver import GripperReceiver


def _packet(
    *,
    sequence: int = 0,
    stream_id: str = "stream-a",
    left: GripperSample | None = None,
    right: GripperSample | None = None,
    button: ButtonSample | None = None,
) -> bytes:
    return encode_packet(
        GripperPacket(
            sequence=sequence,
            sender="umi-pi",
            stream_id=stream_id,
            sent_monotonic_ns=1_000_000_000,
            left=left or GripperSample(120.0, True, (2, 3), 990_000_000),
            right=right or GripperSample(121.0, True, (0, 1), 990_000_000),
            button=button,
        )
    )


def test_receiver_filters_source_orders_packets_and_reports_fresh_samples() -> None:
    receiver = GripperReceiver(sender_ip="10.0.0.2")
    assert not receiver.process_datagram(
        _packet(), source_ip="10.0.0.3", received_at=1.0
    )
    assert receiver.process_datagram(_packet(), source_ip="10.0.0.2", received_at=1.0)
    assert not receiver.process_datagram(
        _packet(), source_ip="10.0.0.2", received_at=1.01
    )

    samples = receiver.snapshot(now=1.05, stale_after_seconds=0.2)

    assert samples["left"].width_mm == 120.0
    assert samples["right"].visible_tag_ids == (0, 1)
    stats = receiver.stats()
    assert stats.packets == 1
    assert stats.wrong_source == 1
    assert stats.out_of_order == 1


def test_receiver_keeps_each_last_valid_width_but_enforces_total_age() -> None:
    receiver = GripperReceiver(sender_ip="10.0.0.2")
    receiver.process_datagram(_packet(), source_ip="10.0.0.2", received_at=1.0)
    receiver.process_datagram(
        _packet(sequence=1, left=GripperSample.invalid(visible_tag_ids=(2,))),
        source_ip="10.0.0.2",
        received_at=1.1,
    )

    samples, errors = receiver.read(now=1.15, stale_after_seconds=0.2)
    assert set(samples) == {"left", "right"}
    assert errors == {}
    assert samples["left"].width_mm == 120.0

    samples, errors = receiver.read(now=1.21, stale_after_seconds=0.2)
    assert "left" not in samples
    assert "left gripper width is stale" in errors["left"]
    assert "right" in samples


def test_new_stream_clears_old_widths_and_allows_sequence_reset() -> None:
    receiver = GripperReceiver(sender_ip="10.0.0.2")
    receiver.process_datagram(
        _packet(sequence=8), source_ip="10.0.0.2", received_at=1.0
    )
    receiver.process_datagram(
        _packet(
            sequence=0,
            stream_id="stream-b",
            left=GripperSample.invalid(),
        ),
        source_ip="10.0.0.2",
        received_at=1.1,
    )

    with pytest.raises(RuntimeError, match="left"):
        receiver.snapshot(now=1.11, stale_after_seconds=0.2)


def test_receiver_rejects_malformed_nested_sample() -> None:
    receiver = GripperReceiver(sender_ip="10.0.0.2")
    data = _packet().replace(b'"width_mm":120.0', b'"width_mm":"bad"')

    assert not receiver.process_datagram(data, source_ip="10.0.0.2", received_at=1.0)
    assert receiver.stats().malformed == 1


def test_button_counters_deliver_each_event_once_despite_packet_loss() -> None:
    receiver = GripperReceiver(sender_ip="10.0.0.2")
    receiver.process_datagram(
        _packet(button=ButtonSample(False, 0, 0)),
        source_ip="10.0.0.2",
        received_at=1.0,
    )
    receiver.process_datagram(
        _packet(sequence=3, button=ButtonSample(True, 3, 1)),
        source_ip="10.0.0.2",
        received_at=1.1,
    )

    events = receiver.poll_button_events(now=1.11, stale_after_seconds=0.2)
    assert events.engage_count == 3
    assert events.quit_count == 1
    assert events.pressed is True
    assert events.stream_id == "stream-a"

    repeated = receiver.poll_button_events(now=1.12, stale_after_seconds=0.2)
    assert repeated.engage_count == 0
    assert repeated.quit_count == 0


def test_new_button_stream_is_baselined_without_replaying_old_actions() -> None:
    receiver = GripperReceiver(sender_ip="10.0.0.2")
    receiver.process_datagram(
        _packet(button=ButtonSample(False, 0, 0)),
        source_ip="10.0.0.2",
        received_at=1.0,
    )
    receiver.process_datagram(
        _packet(
            stream_id="stream-b",
            button=ButtonSample(False, 8, 4),
        ),
        source_ip="10.0.0.2",
        received_at=1.1,
    )

    events = receiver.poll_button_events(now=1.11, stale_after_seconds=0.2)
    assert events.stream_id == "stream-b"
    assert events.engage_count == 0
    assert events.quit_count == 0


def test_button_receiver_rejects_decreasing_counter_and_staleness() -> None:
    receiver = GripperReceiver(sender_ip="10.0.0.2")
    receiver.process_datagram(
        _packet(button=ButtonSample(False, 2, 1)),
        source_ip="10.0.0.2",
        received_at=1.0,
    )
    assert not receiver.process_datagram(
        _packet(sequence=1, button=ButtonSample(False, 1, 1)),
        source_ip="10.0.0.2",
        received_at=1.1,
    )
    assert receiver.stats().malformed == 1

    with pytest.raises(RuntimeError, match="stale"):
        receiver.poll_button_events(now=1.3, stale_after_seconds=0.2)

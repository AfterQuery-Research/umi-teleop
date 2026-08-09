import json

import pytest

from umi_teleop_fixed.gripper_protocol import (
    ButtonSample,
    GripperPacket,
    GripperSample,
    decode_packet,
    encode_packet,
)


def test_gripper_packet_round_trip_preserves_raw_widths_and_validity() -> None:
    packet = GripperPacket(
        sequence=7,
        sender="umi-pi",
        stream_id="test-stream",
        sent_monotonic_ns=2_000_000_000,
        left=GripperSample(124.25, True, (2, 3), 1_990_000_000),
        right=GripperSample.invalid(
            visible_tag_ids=(0,),
            captured_monotonic_ns=1_995_000_000,
        ),
        button=ButtonSample(pressed=True, engage_count=2, quit_count=1),
    )

    payload = decode_packet(encode_packet(packet))

    assert payload["sequence"] == 7
    assert payload["stream_id"] == "test-stream"
    assert payload["left"]["width_mm"] == 124.25
    assert payload["left"]["age_ms"] == 10.0
    assert payload["right"]["valid"] is False
    assert payload["right"]["width_mm"] is None
    assert payload["right"]["visible_tag_ids"] == [0]
    assert payload["button"] == {
        "pressed": True,
        "engage_count": 2,
        "quit_count": 1,
    }


def test_packet_encoding_rejects_invalid_numeric_contract() -> None:
    packet = GripperPacket(
        sequence=0,
        sender="umi-pi",
        stream_id="test-stream",
        sent_monotonic_ns=1,
        left=GripperSample(float("nan"), True, (2, 3), 1),
        right=GripperSample.invalid(),
    )

    with pytest.raises(ValueError, match="finite width_mm"):
        encode_packet(packet)


def test_decode_rejects_unknown_protocol_version() -> None:
    with pytest.raises(ValueError, match="unsupported"):
        decode_packet(json.dumps({"version": 99}).encode())


def test_packet_encoding_rejects_invalid_button_counter() -> None:
    packet = GripperPacket(
        sequence=0,
        sender="umi-pi",
        stream_id="test-stream",
        sent_monotonic_ns=1,
        left=GripperSample.invalid(),
        right=GripperSample.invalid(),
        button=ButtonSample(pressed=False, engage_count=-1, quit_count=0),
    )

    with pytest.raises(ValueError, match="engage_count"):
        encode_packet(packet)

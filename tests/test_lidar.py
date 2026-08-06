from __future__ import annotations

import json
import socket
import time

import numpy as np
import pytest

from umi_teleop.lidar import (
    GLIM_TO_UMI,
    LidarArmReceiver,
    LidarPoseReceiver,
    decode_lidar_arm_packet,
    decode_lidar_pose_packet,
    encode_lidar_arm_packet,
    encode_lidar_pose_packet,
    glim_odom_to_umi_pose,
)
from umi_teleop.lidar_merge_source import _events_from_source, _SourceCounters
from umi_teleop.types import Event


def test_glim_pose_converts_to_umi_handle_frame_without_tcp_shift():
    pose = glim_odom_to_umi_pose([1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(pose[:3], GLIM_TO_UMI @ np.array([1.0, 2.0, 3.0]))
    np.testing.assert_allclose(pose[3:], [0.0, 0.0, 0.0, 1.0])


def test_glim_pose_can_optionally_shift_to_tcp_for_debug_display():
    body = np.array(glim_odom_to_umi_pose([0, 0, 0], [0, 0, 0, 1])[:3])
    tcp = np.array(
        glim_odom_to_umi_pose(
            [0, 0, 0],
            [0, 0, 0, 1],
            tip_offset_m=(-0.10479, 0.0, 0.22244),
        )[:3]
    )
    assert np.linalg.norm(tcp - body) == pytest.approx(0.2459, rel=1e-3)


def test_lidar_pose_packet_round_trips_and_rejects_bad_quaternion():
    packet = {
        "version": 1,
        "source": "lambda-glim",
        "seq": 7,
        "arms": {
            "left": {
                "serial": "pi-a",
                "pose": [0, 0, 0, 0, 0, 0, 1],
                "source_time_s": 123.0,
            }
        },
    }
    assert decode_lidar_pose_packet(encode_lidar_pose_packet(packet)) == packet
    packet["arms"]["left"]["pose"][3:] = [0, 0, 0, 2]
    with pytest.raises(ValueError, match="normalized"):
        encode_lidar_pose_packet(packet)


def test_lidar_arm_packet_round_trips_and_requires_consistent_width_age():
    packet = {
        "version": 1,
        "session_id": "s",
        "source_id": "leader",
        "arm": "right",
        "seq": 3,
        "requested_enabled": False,
        "zero_id": 0,
        "toggle_id": 1,
        "cancel_id": 0,
        "quit_id": 0,
        "width_mm": 81.5,
        "width_age_s": 0.02,
    }
    assert decode_lidar_arm_packet(encode_lidar_arm_packet(packet)) == packet
    packet["width_mm"] = None
    with pytest.raises(ValueError, match="width_age_s"):
        encode_lidar_arm_packet(packet)


def test_lidar_receivers_expose_latest_samples_with_local_arrival_age():
    pose_receiver = LidarPoseReceiver("127.0.0.1", 0)
    arm_receiver = LidarArmReceiver("127.0.0.1", 0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    pose_receiver.start()
    arm_receiver.start()
    try:
        pose_packet = {
            "version": 1,
            "seq": 1,
            "arms": {
                "left": {
                    "serial": "left-pi",
                    "pose": [0, 0, 0, 0, 0, 0, 1],
                    "source_time_s": 1.0,
                }
            },
        }
        arm_packet = {
            "version": 1,
            "session_id": "s",
            "source_id": "left-pi",
            "arm": "left",
            "seq": 1,
            "requested_enabled": False,
            "zero_id": 0,
            "toggle_id": 0,
            "cancel_id": 0,
            "quit_id": 0,
            "width_mm": 42.0,
            "width_age_s": 0.01,
        }
        sock.sendto(encode_lidar_pose_packet(pose_packet), ("127.0.0.1", pose_receiver.port))
        sock.sendto(encode_lidar_arm_packet(arm_packet), ("127.0.0.1", arm_receiver.port))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            poses = pose_receiver.snapshot()
            widths = arm_receiver.widths()
            if "left" in poses and widths["left"]["width_mm"] is not None:
                assert poses["left"]["serial"] == "left-pi"
                assert poses["left"]["age_s"] >= 0.0
                assert widths["left"]["width_mm"] == 42.0
                assert widths["right"]["width_mm"] is None
                break
            time.sleep(0.01)
        else:
            raise AssertionError("receivers did not update")
    finally:
        pose_receiver.stop()
        arm_receiver.stop()
        sock.close()


def test_lidar_arm_receiver_accepts_restarted_source_session_with_reset_sequence():
    arm_receiver = LidarArmReceiver("127.0.0.1", 0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    arm_receiver.start()
    try:
        base = {
            "version": 1,
            "source_id": "right-pi",
            "arm": "right",
            "requested_enabled": False,
            "zero_id": 0,
            "toggle_id": 0,
            "cancel_id": 0,
            "quit_id": 0,
            "width_age_s": 0.01,
        }
        old = {**base, "session_id": "old", "seq": 100, "width_mm": 40.0}
        new = {**base, "session_id": "new", "seq": 1, "width_mm": 55.0}
        sock.sendto(encode_lidar_arm_packet(old), ("127.0.0.1", arm_receiver.port))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if arm_receiver.widths()["right"]["width_mm"] == 40.0:
                break
            time.sleep(0.01)
        else:
            raise AssertionError("initial arm packet was not received")
        sock.sendto(encode_lidar_arm_packet(new), ("127.0.0.1", arm_receiver.port))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if arm_receiver.widths()["right"]["width_mm"] == 55.0:
                break
            time.sleep(0.01)
        else:
            raise AssertionError("restarted source session with reset seq was ignored")
    finally:
        arm_receiver.stop()
        sock.close()


def test_arm_source_counter_changes_become_global_button_events_once():
    previous = _SourceCounters()
    packet = {
        "session_id": "s",
        "requested_enabled": True,
        "zero_id": 1,
        "toggle_id": 1,
        "cancel_id": 0,
        "quit_id": 0,
    }
    assert _events_from_source(packet, previous) == [Event.ZERO, Event.START]
    assert _events_from_source(packet, previous) == []
    packet = dict(packet, requested_enabled=False, toggle_id=2, cancel_id=1)
    assert _events_from_source(packet, previous) == [Event.CANCEL, Event.STOP]


def test_pose_decoder_rejects_empty_arm_set():
    with pytest.raises(ValueError, match="at least one arm"):
        decode_lidar_pose_packet(json.dumps({"version": 1, "seq": 1, "arms": {}}).encode())

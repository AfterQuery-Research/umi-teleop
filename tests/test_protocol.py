from __future__ import annotations

import json
import socket
import time

import numpy as np
import pytest

from umi_teleop.buttons import KeyboardButtons
from umi_teleop.protocol import (
    ButtonState,
    EnableRequestGuard,
    VivePacketReceiver,
    WidthWorker,
    ZeroRequestGuard,
    decode_teleop_packet,
    encode_teleop_packet,
)
from umi_teleop.source import DATA_DIR, Config
from umi_teleop.types import Event, Frame


def _arms():
    return {
        "left": {
            "serial": "A",
            "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "pose_age_s": 0.01,
            "width_mm": 20.0,
            "width_age_s": 0.02,
        },
        "right": {
            "serial": "B",
            "pose": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            "pose_age_s": 0.01,
            "width_mm": None,
            "width_age_s": None,
        },
    }


def _packet():
    return {
        "version": 1,
        "session_id": "test",
        "seq": 3,
        "requested_enabled": False,
        "zero_id": 1,
        "toggle_id": 2,
        "cancel_id": 0,
        "quit_id": 0,
        "arms": _arms(),
    }


def test_default_side_mapping_keeps_each_physical_umi_paired():
    cfg = Config()
    assert (
        cfg.right_serial,
        cfg.right_device,
        cfg.right_calib,
        cfg.right_tag_ids,
    ) == (
        "LHR-C891E882",
        "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0",
        str(DATA_DIR / "umi1_calib.npz"),
        (0, 1),
    )
    assert (
        cfg.left_serial,
        cfg.left_device,
        cfg.left_calib,
        cfg.left_tag_ids,
    ) == (
        "LHR-BBAF093A",
        "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:2:1.0-video-index0",
        str(DATA_DIR / "umi2_calib.npz"),
        (2, 3),
    )
    assert cfg.zero_timeout_s == 4.0
    assert cfg.target_host == "127.0.0.1"
    assert not cfg.arm_only


def test_keyboard_e_toggles_enable():
    assert KeyboardButtons.KEYMAP["e"] is Event.TOGGLE


def test_packet_round_trip_and_rejects_nonfinite_or_bad_quaternion():
    assert decode_teleop_packet(encode_teleop_packet(_packet())) == _packet()
    bad = _packet()
    bad["arms"]["left"]["pose"][0] = float("nan")
    with pytest.raises((ValueError, TypeError)):
        encode_teleop_packet(bad)
    bad = _packet()
    bad["arms"]["left"]["pose"][3:] = [0.0, 0.0, 0.0, 2.0]
    with pytest.raises(ValueError, match="normalized"):
        encode_teleop_packet(bad)


def test_button_state_zero_disarms_and_counters_survive_packets():
    state = ButtonState(session_id="buttons")
    state.handle(Event.TOGGLE)
    assert state.requested_enabled
    state.handle(Event.ZERO)
    assert not state.requested_enabled and state.zero_id == 1
    state.handle(Event.TOGGLE)
    state.handle(Event.CANCEL)
    assert not state.requested_enabled and state.cancel_id == 1
    state.handle(Event.QUIT)
    packet = state.packet(7, {}, {})
    assert state.quitting and packet["quit_id"] == 1
    assert packet["arms"]["left"]["pose"] is None


def test_enable_guard_requires_a_quiet_period_and_fresh_inputs():
    state = ButtonState(session_id="guard")
    guard = EnableRequestGuard(delay_s=3.0, cooldown_s=5.0)

    assert guard.handle(state, Event.TOGGLE, 10.0) == "enable pending for 3.0s"
    assert guard.pending and not state.requested_enabled and state.toggle_id == 0
    assert guard.update(state, 12.9, inputs_fresh=True) is None
    assert guard.update(state, 13.0, inputs_fresh=True) == "enable confirmed after quiet period"
    assert state.requested_enabled and state.toggle_id == 1

    guard.handle(state, Event.TOGGLE, 14.0)
    assert not state.requested_enabled and state.toggle_id == 2
    assert "cooldown" in guard.handle(state, Event.TOGGLE, 15.0)
    assert guard.update(state, 30.0, inputs_fresh=True) is None
    assert not state.requested_enabled

    guard.handle(state, Event.TOGGLE, 20.0)
    assert guard.update(state, 23.0, inputs_fresh=False) == (
        "enable rejected because pose or width telemetry is stale"
    )
    assert not state.requested_enabled and state.cancel_id == 1


def test_enable_guard_rejects_toggle_noise_around_a_hold_cancel():
    state = ButtonState(session_id="noisy-hold")
    guard = EnableRequestGuard(delay_s=3.0, cooldown_s=5.0)

    # Reproduce the real GPIO trace: release-toggle, another release-toggle two seconds later,
    # and finally the hold callback. The first event can only disarm; later bounce is ignored.
    state.handle(Event.START)
    guard.handle(state, Event.TOGGLE, 30.0)
    assert not state.requested_enabled
    assert "cooldown" in guard.handle(state, Event.TOGGLE, 32.0)
    guard.handle(state, Event.CANCEL, 34.0)
    assert "cooldown" in guard.handle(state, Event.TOGGLE, 36.0)
    assert guard.update(state, 50.0, inputs_fresh=True) is None
    assert not state.requested_enabled and not guard.pending

    # The same contact noise while initially disabled is also fail-closed.
    fresh_state = ButtonState(session_id="noisy-disabled")
    fresh_guard = EnableRequestGuard(delay_s=3.0, cooldown_s=5.0)
    fresh_guard.handle(fresh_state, Event.TOGGLE, 60.0)
    fresh_guard.handle(fresh_state, Event.CANCEL, 61.0)
    assert "cooldown" in fresh_guard.handle(fresh_state, Event.TOGGLE, 62.0)
    assert fresh_guard.update(fresh_state, 80.0, inputs_fresh=True) is None
    assert not fresh_state.requested_enabled and not fresh_guard.pending


def test_zero_guard_waits_for_occlusion_to_clear_then_counts_once():
    state = ButtonState(session_id="deferred-zero")
    state.handle(Event.START)
    guard = ZeroRequestGuard(settle_s=0.4, timeout_s=2.0)

    assert guard.request(state, 10.0) == "zero pending after 0.4s settle"
    assert not state.requested_enabled and state.zero_id == 0 and guard.pending
    assert guard.update(state, 10.3, poses_fresh=True) is None
    assert guard.update(state, 10.5, poses_fresh=False) is None
    assert guard.update(state, 11.5, poses_fresh=True) == ("zero confirmed with both fresh poses")
    assert state.zero_id == 1 and not guard.pending
    assert guard.update(state, 20.0, poses_fresh=True) is None


def test_zero_guard_rejects_when_tracking_does_not_recover():
    state = ButtonState(session_id="zero-timeout")
    guard = ZeroRequestGuard(settle_s=0.4, timeout_s=2.0)
    guard.request(state, 30.0)

    assert guard.update(state, 32.0, poses_fresh=False) == (
        "zero rejected because both poses did not recover before timeout"
    )
    assert state.zero_id == 0 and not guard.pending


def test_vive_receiver_maps_both_serials_and_ages():
    receiver = VivePacketReceiver({"A": "left", "B": "right"}, host="127.0.0.1", port=0)
    receiver.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        payload = {
            "trackers": [
                {"serial": "A", "valid": True, "pos": [0, 0, 0], "quat": [1, 0, 0, 0]},
                {"serial": "B", "valid": True, "pos": [1, 0, 0], "quat": [1, 0, 0, 0]},
            ]
        }
        sender.sendto(json.dumps(payload).encode(), ("127.0.0.1", receiver.port))
        deadline = time.time() + 1.0
        while receiver.packet_count == 0 and time.time() < deadline:
            time.sleep(0.01)
        snap = receiver.snapshot()
        assert set(snap) == {"left", "right"}
        assert snap["left"]["serial"] == "A"
        assert snap["right"]["pose"][:3] == [0.0, -1.0, 0.0]
        assert snap["left"]["age_s"] >= 0.0
    finally:
        sender.close()
        receiver.stop()


def test_native_vive_receiver_propagates_sensor_age_and_clears_invalid_pose():
    receiver = VivePacketReceiver({"A": "left", "B": "right"}, host="127.0.0.1", port=0)
    receiver.start()
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        valid = {
            "frame": "libsurvive_tracking",
            "trackers": [
                {
                    "serial": "A",
                    "valid": True,
                    "pose_age_s": 0.08,
                    "pos": [0, 0, 0],
                    "quat": [1, 0, 0, 0],
                }
            ],
        }
        sender.sendto(json.dumps(valid).encode(), ("127.0.0.1", receiver.port))
        deadline = time.time() + 1.0
        while receiver.packet_count < 1 and time.time() < deadline:
            time.sleep(0.01)
        assert receiver.snapshot()["left"]["age_s"] >= 0.075

        invalid = {
            "frame": "libsurvive_tracking",
            "trackers": [{"serial": "A", "valid": False}],
        }
        sender.sendto(json.dumps(invalid).encode(), ("127.0.0.1", receiver.port))
        deadline = time.time() + 1.0
        while receiver.packet_count < 2 and time.time() < deadline:
            time.sleep(0.01)
        assert "left" not in receiver.snapshot()
    finally:
        sender.close()
        receiver.stop()


class _Camera:
    def __init__(self):
        self.started = False
        self.stopped = False
        self.sent = False

    def start(self):
        self.started = True

    def read(self):
        if self.sent:
            return []
        self.sent = True
        return [Frame(timestamp=time.time(), image=np.zeros((4, 4, 3), np.uint8))]

    def stop(self):
        self.stopped = True


class _Detector:
    def width_mm(self, _image):
        return 42.5


def test_width_worker_retains_latest_successful_width():
    camera = _Camera()
    worker = WidthWorker(camera, _Detector(), detect_hz=100.0)
    worker.start()
    try:
        deadline = time.time() + 1.0
        while worker.snapshot()["width_mm"] is None and time.time() < deadline:
            time.sleep(0.01)
        snap = worker.snapshot()
        assert snap["width_mm"] == pytest.approx(42.5)
        assert snap["age_s"] >= 0.0
    finally:
        worker.stop()
    assert camera.started and camera.stopped

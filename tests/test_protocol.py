from __future__ import annotations

import io
import json
import socket
import time

import numpy as np
import pytest

from umi_teleop import buttons as buttons_module
from umi_teleop import source as source_module
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


def test_keyboard_has_deterministic_start_and_stop_keys():
    assert KeyboardButtons.KEYMAP["e"] is Event.START
    assert KeyboardButtons.KEYMAP["s"] is Event.STOP


def test_keyboard_closed_stdin_quits_once(monkeypatch):
    stdin = io.StringIO("")
    monkeypatch.setattr(buttons_module.sys, "stdin", stdin)
    monkeypatch.setattr(buttons_module.select, "select", lambda *_: ([stdin], [], []))
    buttons = KeyboardButtons()

    assert buttons.poll() is Event.QUIT
    assert buttons.poll() is None


def test_source_main_treats_launcher_interrupt_as_clean_shutdown(monkeypatch):
    monkeypatch.setattr(source_module.tyro, "cli", lambda *_: Config())

    def interrupt(_config):
        raise KeyboardInterrupt

    monkeypatch.setattr(source_module, "run", interrupt)

    assert source_module.main() is None


def test_tracking_status_identifies_missing_side_and_serial():
    status = source_module._tracking_status(
        {"left": {"age_s": 0.012}},
        {"left": "LHR-LEFT", "right": "LHR-RIGHT"},
    )

    assert status == "left(LHR-LEFT)=age=0.012s right(LHR-RIGHT)=missing"


def test_vive_continuity_guard_detects_generation_change_only_with_both_trackers():
    guard = source_module._ViveContinuityGuard()
    poses = {
        "left": {"continuity_id": 1},
        "right": {"continuity_id": 2},
    }
    assert not guard.update(poses)
    assert not guard.update(poses)
    assert not guard.update({"left": {"continuity_id": 3}})
    changed = {"left": {"continuity_id": 3}, "right": {"continuity_id": 2}}
    assert guard.update(changed)
    assert not guard.update(changed)


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


def test_pi_event_consumer_baselines_history_and_preserves_new_safety_events():
    consumer = source_module._PiEventConsumer()
    packet = {
        "session_id": "pi-a",
        "events": {
            "zero": 4,
            "toggle": 3,
            "start": 2,
            "stop": 1,
            "cancel": 0,
            "quit": 0,
        },
    }
    changed, events = consumer.update(packet)
    assert changed and events == []

    packet["events"] = dict(packet["events"], toggle=4, stop=2)
    changed, events = consumer.update(packet)
    assert not changed and events == [Event.STOP]

    packet["events"] = dict(packet["events"], toggle=5, start=3, cancel=1)
    _changed, events = consumer.update(packet)
    assert events == [Event.CANCEL]

    packet["events"] = dict(packet["events"], toggle=7, start=4, stop=3)
    _changed, events = consumer.update(packet)
    assert events == [Event.CANCEL]


def test_pi_event_consumer_preserves_stop_intent_after_local_safety_disarm():
    consumer = source_module._PiEventConsumer()
    packet = {
        "session_id": "pi-stop-after-disarm",
        "events": {
            "zero": 1,
            "toggle": 1,
            "start": 1,
            "stop": 0,
            "cancel": 0,
            "quit": 0,
        },
    }
    assert consumer.update(packet) == (True, [])

    packet["events"] = dict(packet["events"], toggle=2, stop=1)
    assert consumer.update(packet) == (False, [Event.STOP])


def test_pi_widths_retain_recent_measurement_during_brief_tag_miss():
    arm = {
        "width_mm": 81.5,
        "width_age_s": 0.20,
        "tags_visible": False,
        "camera_ok": True,
    }
    packet = {"arms": {"left": dict(arm), "right": dict(arm)}}

    widths = source_module._pi_widths(packet, 0.03, heartbeat_timeout_s=0.5)

    assert widths == {
        "left": {"width_mm": 81.5, "age_s": pytest.approx(0.23)},
        "right": {"width_mm": 81.5, "age_s": pytest.approx(0.23)},
    }


def test_pi_widths_reject_stale_heartbeat_or_unhealthy_camera():
    arm = {
        "width_mm": 81.5,
        "width_age_s": 0.20,
        "tags_visible": True,
        "camera_ok": True,
    }
    packet = {"arms": {"left": dict(arm), "right": dict(arm, camera_ok=False)}}

    stale = source_module._pi_widths(packet, 0.51, heartbeat_timeout_s=0.5)
    unhealthy = source_module._pi_widths(packet, 0.03, heartbeat_timeout_s=0.5)

    assert all(value == {"width_mm": None, "age_s": None} for value in stale.values())
    assert unhealthy["left"]["width_mm"] == 81.5
    assert unhealthy["right"] == {"width_mm": None, "age_s": None}


def test_stale_input_status_names_each_failed_freshness_gate():
    cfg = Config(max_pose_age_s=0.25, max_width_age_s=0.5, pi_heartbeat_timeout_s=0.4)
    poses = {"left": {"age_s": 0.31}, "right": {"age_s": 0.02}}
    widths = {"left": {"age_s": 0.72}, "right": {"age_s": 0.03}}

    status = source_module._stale_input_status(
        cfg,
        poses,
        widths,
        poses_fresh=False,
        widths_fresh=False,
        pi_fresh=False,
        pi_age=0.65,
    )

    assert "Vive pose age[left=0.310s right=0.020s] limit=0.250s" in status
    assert "Pi heartbeat age=0.650s limit=0.400s" in status
    assert "ArUco width age[left=0.720s right=0.030s] limit=0.500s" in status


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
                    "continuity_id": 4,
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
        assert receiver.snapshot()["left"]["continuity_id"] == 4

        invalid = {
            "frame": "libsurvive_tracking",
            "trackers": [{"serial": "A", "valid": False, "continuity_id": 5}],
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

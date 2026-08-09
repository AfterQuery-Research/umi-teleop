import json
import math
from dataclasses import dataclass

import numpy as np
import pytest

from umi_teleop_fixed import teleop
from umi_teleop_fixed.calibration import UmiGripperCalibration
from umi_teleop_fixed.gripper_receiver import ReceivedButtonEvents, ReceivedGripperSample
from umi_teleop_fixed.robot import DEFAULT_START_POSITION, ZERO_POSITION, ArmState
from umi_teleop_fixed.teleop import Args, TeleopController, control_loop, run
from umi_teleop_fixed.trackers import TrackerPose, TrackerSample


def _args(mode="dry-run", **overrides):
    values = dict(
        robot_id="robot",
        left_adapter_serial="LEFT-CAN",
        right_adapter_serial="RIGHT-CAN",
        left_tracker_serial="LEFT-TRACKER",
        right_tracker_serial="RIGHT-TRACKER",
        tracker_to_robot_rpy_degrees=(0.0, 0.0, 0.0),
        mode=mode,
    )
    values.update(overrides)
    return Args(**values)


def _samples(x=0.0, received_at=1.0):
    return {
        side: TrackerSample(
            TrackerPose(
                name=side,
                serial=f"{side}-tracker",
                timestamp=1.0 + x,
                position=(x, 0.0, 0.0),
                quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            ),
            received_at,
        )
        for side in ("left", "right")
    }


def _states():
    return {side: ArmState(np.zeros(6), 0.4, np.eye(4), 1.0) for side in ("left", "right")}


def _gripper_samples(left=120.0, right=120.0, *, stream_id="stream-a", received_at=1.0):
    return {
        side: ReceivedGripperSample(
            width_mm=left if side == "left" else right,
            visible_tag_ids=(2, 3) if side == "left" else (0, 1),
            received_at=received_at,
            camera_age_seconds=0.01,
            sequence=1,
            stream_id=stream_id,
            sender="umi-pi",
            source_ip="10.0.0.2",
        )
        for side in ("left", "right")
    }


def _umi_calibrations(span_mm=80.0):
    return {
        side: UmiGripperCalibration(
            sender_ip="10.0.0.2",
            closed_width_mm=40.0,
            open_width_mm=40.0 + span_mm,
            calibrated_at_utc="2026-08-08T12:00:00Z",
        )
        for side in ("left", "right")
    }


def _button_events(
    engage=0,
    quit=0,
    *,
    stream_id="stream-a",
    pressed=False,
):
    return ReceivedButtonEvents(
        engage_count=engage,
        quit_count=quit,
        pressed=pressed,
        stream_id=stream_id,
        sender="umi-pi",
        source_ip="10.0.0.2",
    )


def test_teleop_defaults_to_scene_solving_poser():
    assert _args().poser == "MPFIT"


def test_teleop_loads_cached_robot_frame_by_default(monkeypatch, tmp_path):
    from test_lighthouses import lighthouse_config

    from umi_teleop_fixed.lighthouses import save_new_lighthouse_calibration_id
    from umi_teleop_fixed.teleop_frame import (
        TeleopFrameCalibration,
        save_teleop_frame,
        teleop_frame_path,
    )

    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path))
    args = _args(tracker_to_robot_rpy_degrees=None)
    config_path = args.resolved_config_path()
    config_path.parent.mkdir(parents=True)
    config_path.write_text(lighthouse_config())
    calibration_id = save_new_lighthouse_calibration_id(config_path)
    path = teleop_frame_path("robot")
    save_teleop_frame(
        path,
        robot_id="robot",
        calibration=TeleopFrameCalibration(
            rpy_degrees=(0.0, 0.0, -150.0),
            calibrated_at_utc="2026-08-08T23:40:00+00:00",
            source="test",
            lighthouse_calibration_id=calibration_id,
        ),
    )

    assert args.resolved_tracker_to_robot_rpy_degrees() == (0.0, 0.0, -150.0)


def test_explicit_robot_frame_does_not_require_cached_calibration(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path))

    args = _args(tracker_to_robot_rpy_degrees=(0.0, 0.0, 30.0))

    assert args.resolved_tracker_to_robot_rpy_degrees() == (0.0, 0.0, 30.0)


def test_teleop_requires_and_freezes_lighthouse_calibration(monkeypatch, tmp_path):
    captured = {}
    checked = []

    class Source:
        def __init__(self, **kwargs):
            captured.update(kwargs)

    monkeypatch.setattr(teleop, "require_lighthouse_calibration", checked.append)
    monkeypatch.setattr(teleop, "LibsurviveSource", Source)
    config_path = tmp_path / "config.json"

    teleop._create_source(_args(config_path=config_path))

    assert checked == [config_path]
    assert captured["disable_calibration"] is True


def test_controller_captures_a_no_jump_baseline_and_rate_limits():
    controller = TeleopController(_args(control_rate_hz=10.0, max_linear_speed_m_s=0.2))
    controller.engage(_states(), _samples())

    initial = controller.propose(_samples(), dt=0.1)
    moved = controller.propose_detailed(_samples(x=0.1), dt=0.1)

    assert initial["left"] == pytest.approx(np.eye(4))
    assert moved.command["left"][0, 3] == pytest.approx(0.02)
    controller.commit(
        moved.command,
        controller.seeds(),
        requested_targets=moved.requested,
    )
    summary = controller.motion_summary()
    assert "left request_mm=[+100.0,+0.0,+0.0]" in summary
    assert "command_mm=[+20.0,+0.0,+0.0]" in summary


def test_gripper_mapping_is_engage_relative_clamped_and_rate_limited():
    controller = TeleopController(
        _args(
            gripper_sender_ip="10.0.0.2",
            max_gripper_speed_per_s=1.5,
        ),
        gripper_calibrations=_umi_calibrations(),
    )
    controller.engage(_states(), _samples(), _gripper_samples())

    initial = controller.propose_grippers(_gripper_samples(), dt=0.1)
    moved = controller.propose_grippers(_gripper_samples(left=140.0, right=80.0), dt=0.1)

    assert initial.command == pytest.approx({"left": 0.4, "right": 0.4})
    assert moved.requested == pytest.approx({"left": 0.65, "right": 0.0})
    assert moved.command == pytest.approx({"left": 0.55, "right": 0.25})


def test_gripper_stream_restart_latches_until_rezero():
    controller = TeleopController(
        _args(gripper_sender_ip="10.0.0.2"),
        gripper_calibrations=_umi_calibrations(),
    )
    controller.engage(_states(), _samples(), _gripper_samples())

    restarted = controller.propose_grippers(
        _gripper_samples(left=140.0, stream_id="stream-b"), dt=0.1
    )
    recovered = controller.propose_grippers(_gripper_samples(left=140.0), dt=0.1)

    assert restarted.command["left"] == 0.4
    assert "re-zero required" in recovered.status["left"]


def test_ordinary_gripper_staleness_holds_then_automatically_recovers():
    controller = TeleopController(
        _args(gripper_sender_ip="10.0.0.2"),
        gripper_calibrations=_umi_calibrations(),
    )
    controller.engage(_states(), _samples(), _gripper_samples())

    held = controller.propose_grippers(
        {},
        dt=0.1,
        unavailable_reasons={"left": "stale", "right": "stale"},
    )
    recovered = controller.propose_grippers(_gripper_samples(left=128.0), dt=0.1)

    assert held.command == {"left": 0.4, "right": 0.4}
    assert recovered.status["left"] == "tracking"
    assert recovered.command["left"] > 0.4


def test_controller_applies_fixed_tracker_to_robot_basis():
    controller = TeleopController(
        _args(
            control_rate_hz=10.0,
            max_linear_speed_m_s=1.0,
            tracker_to_robot_rpy_degrees=(0.0, 0.0, 90.0),
        )
    )
    controller.engage(_states(), _samples())

    moved = controller.propose(_samples(x=0.1), dt=0.1)

    assert moved["left"][:3, 3] == pytest.approx((0.0, 0.1, 0.0), abs=1e-9)


def test_controller_rejects_target_outside_engage_workspace():
    controller = TeleopController(_args(max_translation_from_start_m=0.3))
    controller.engage(_states(), _samples())

    with pytest.raises(RuntimeError, match="safe workspace"):
        controller.propose(_samples(x=0.31), dt=1 / 60)


def test_controller_compensates_steady_joint_tracking_error_with_a_bound():
    controller = TeleopController(_args())
    states = _states()
    controller.engage(states, _samples())
    desired = {side: np.full(6, 0.1) for side in ("left", "right")}

    commands = controller.compensated_joint_commands(desired, states, dt=1.0)

    assert commands["left"] == pytest.approx(np.full(6, 0.15))
    assert controller.joint_corrections()["right"] == pytest.approx(np.full(6, 0.05))


def test_controller_checks_the_measured_physical_tcp_workspace():
    controller = TeleopController(_args(max_translation_from_start_m=0.3))
    controller.engage(_states(), _samples())
    moved = _states()
    moved["right"].ee_pose[0, 3] = 0.31

    with pytest.raises(RuntimeError, match="measured physical TCP"):
        controller.validate_measured_workspace(moved)


@dataclass
class FakeKeys:
    batches: list[tuple[str, ...]]

    def poll(self):
        return self.batches.pop(0) if self.batches else ()


class FakeTrackers:
    def __init__(self, *, fail_after=None):
        self.calls = 0
        self.fail_after = fail_after

    def snapshot(self, **_kwargs):
        self.calls += 1
        if self.fail_after is not None and self.calls >= self.fail_after:
            raise RuntimeError("left tracker pose is stale")
        return _samples(received_at=0.0)


class FakeGrippers:
    def __init__(self, *, baseline=None, current=None, errors=None, button_events=None):
        self.baseline = baseline or _gripper_samples(received_at=0.0)
        self.current = current or self.baseline
        self.errors = errors or {}
        self.button_events = list(button_events or [])

    def snapshot(self, **_kwargs):
        if isinstance(self.baseline, Exception):
            raise self.baseline
        return self.baseline

    def read(self, **_kwargs):
        return self.current, self.errors

    def poll_button_events(self, **_kwargs):
        if self.button_events:
            event = self.button_events.pop(0)
            if isinstance(event, Exception):
                raise event
            return event
        return _button_events()


class FakeBackend:
    def __init__(self):
        self.commands = []
        self.idle_count = 0
        self.read_count = 0
        self.move_calls = []

    def read_states(self):
        self.read_count += 1
        return _states()

    def solve_ik(self, _targets, seeds):
        return {side: value.copy() for side, value in seeds.items()}

    def command(self, joints, grippers):
        self.commands.append((joints, grippers))

    def idle(self):
        self.idle_count += 1

    def move_to_position(self, position, **_kwargs):
        self.move_calls.append(tuple(position))

    def __enter__(self):
        return self

    def __exit__(self, *_exc_info):
        self.closed = True


def test_dry_run_executes_both_ik_paths_but_never_commands():
    backend = FakeBackend()
    output = []
    control_loop(
        _args("dry-run"),
        backend=backend,
        trackers=FakeTrackers(),
        keys=FakeKeys([("e",), (), ("q",)]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=output.append,
    )

    assert backend.read_count >= 2
    assert backend.commands == []
    assert any("ENGAGED" in line for line in output)
    assert backend.idle_count == 2


def test_active_mode_commands_only_after_both_ik_solutions():
    backend = FakeBackend()
    control_loop(
        _args("active"),
        backend=backend,
        trackers=FakeTrackers(),
        keys=FakeKeys([("e",), ("q",)]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=lambda _line: None,
    )

    assert len(backend.commands) == 1
    assert set(backend.commands[0][0]) == {"left", "right"}


def test_active_mode_commands_relative_streamed_gripper_widths():
    backend = FakeBackend()
    control_loop(
        _args(
            "active",
            gripper_sender_ip="10.0.0.2",
            max_gripper_speed_per_s=60.0,
        ),
        backend=backend,
        trackers=FakeTrackers(),
        grippers=FakeGrippers(current=_gripper_samples(left=128.0, right=112.0)),
        gripper_calibrations=_umi_calibrations(),
        keys=FakeKeys([("e",), ("q",)]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=lambda _line: None,
    )

    assert backend.commands[0][1] == pytest.approx({"left": 0.5, "right": 0.3})


def test_remote_button_tap_engages_and_hold_quits() -> None:
    backend = FakeBackend()
    output = []
    control_loop(
        _args(
            "active",
            gripper_sender_ip="10.0.0.2",
            enable_remote_button=True,
        ),
        backend=backend,
        trackers=FakeTrackers(),
        grippers=FakeGrippers(
            button_events=[_button_events(engage=1), _button_events(quit=1)],
        ),
        gripper_calibrations=_umi_calibrations(),
        keys=FakeKeys([(), ()]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=output.append,
    )

    assert len(backend.commands) == 1
    assert any("ENGAGED via GPIO button" in line for line in output)


def test_remote_button_loss_disengages_an_active_session() -> None:
    backend = FakeBackend()
    output = []
    control_loop(
        _args(
            "active",
            gripper_sender_ip="10.0.0.2",
            enable_remote_button=True,
        ),
        backend=backend,
        trackers=FakeTrackers(),
        grippers=FakeGrippers(
            button_events=[
                _button_events(engage=1),
                RuntimeError("Pi button stream is stale"),
            ],
        ),
        gripper_calibrations=_umi_calibrations(),
        keys=FakeKeys([(), ()]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=output.append,
        max_cycles=2,
    )

    assert len(backend.commands) == 1
    assert any("remote button became unavailable" in line for line in output)
    assert backend.idle_count == 3


def test_remote_button_staleness_logs_once_until_recovery() -> None:
    backend = FakeBackend()
    output = []
    control_loop(
        _args(
            "dry-run",
            gripper_sender_ip="10.0.0.2",
            enable_remote_button=True,
        ),
        backend=backend,
        trackers=FakeTrackers(),
        grippers=FakeGrippers(
            button_events=[
                RuntimeError("Pi button stream is stale (0.501s)"),
                RuntimeError("Pi button stream is stale (0.518s)"),
            ],
        ),
        gripper_calibrations=_umi_calibrations(),
        keys=FakeKeys([(), ()]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=output.append,
        max_cycles=2,
    )

    assert sum("REMOTE BUTTON UNAVAILABLE" in line for line in output) == 1


def test_remote_button_stream_restart_disengages_without_replaying_events() -> None:
    backend = FakeBackend()
    output = []
    control_loop(
        _args(
            "active",
            gripper_sender_ip="10.0.0.2",
            enable_remote_button=True,
        ),
        backend=backend,
        trackers=FakeTrackers(),
        grippers=FakeGrippers(
            button_events=[
                _button_events(engage=1),
                _button_events(stream_id="stream-b"),
            ],
        ),
        gripper_calibrations=_umi_calibrations(),
        keys=FakeKeys([(), ()]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=output.append,
        max_cycles=2,
    )

    assert len(backend.commands) == 1
    assert any("Pi button stream restarted" in line for line in output)


def test_gripper_dropout_freezes_grippers_without_stopping_arms():
    backend = FakeBackend()
    control_loop(
        _args("active", gripper_sender_ip="10.0.0.2"),
        backend=backend,
        trackers=FakeTrackers(),
        grippers=FakeGrippers(current={}, errors={"left": "stale", "right": "stale"}),
        gripper_calibrations=_umi_calibrations(),
        keys=FakeKeys([("e",), (), ("q",)]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=lambda _line: None,
    )

    assert len(backend.commands) == 2
    assert all(command[1] == {"left": 0.4, "right": 0.4} for command in backend.commands)
    assert backend.idle_count == 2


def test_engage_is_refused_without_fresh_gripper_widths():
    backend = FakeBackend()
    output = []
    control_loop(
        _args("active", gripper_sender_ip="10.0.0.2"),
        backend=backend,
        trackers=FakeTrackers(),
        grippers=FakeGrippers(baseline=RuntimeError("no gripper packets received")),
        gripper_calibrations=_umi_calibrations(),
        keys=FakeKeys([("e",), ("q",)]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=output.append,
    )

    assert backend.commands == []
    assert any("ENGAGE REFUSED" in line for line in output)


def test_stale_tracker_disengages_both_arms():
    backend = FakeBackend()
    output = []

    class Recorder:
        path = None

        def __init__(self):
            self.events = []

        def event(self, event_type, **values):
            self.events.append((event_type, values))

        def close(self):
            pass

    recorder = Recorder()
    control_loop(
        _args("active"),
        backend=backend,
        trackers=FakeTrackers(fail_after=2),
        keys=FakeKeys([("e",), (), ("q",)]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=output.append,
        debug_recorder=recorder,
    )

    assert backend.commands == []
    assert backend.idle_count == 3
    assert any("SAFETY STOP" in line and "stale" in line for line in output)
    assert any(event == "safety_stop" for event, _values in recorder.events)


def test_args_reject_duplicate_hardware_ids_and_nonpositive_limits():
    with pytest.raises(ValueError, match="CAN adapter"):
        _args(right_adapter_serial="left-can").validate()
    with pytest.raises(ValueError, match="tracker serials"):
        _args(right_tracker_serial="left-tracker").validate()
    with pytest.raises(ValueError, match="control_rate_hz"):
        _args(control_rate_hz=0).validate()
    with pytest.raises(ValueError, match="14 finite"):
        _args(start_position=(0.0,) * 13).validate()
    _args(rotation_scale=0.0).validate()
    with pytest.raises(ValueError, match="rotation_scale"):
        _args(rotation_scale=-0.1).validate()
    with pytest.raises(ValueError, match="three finite"):
        _args(tracker_to_robot_rpy_degrees=(0.0, math.nan, 0.0)).validate()
    with pytest.raises(ValueError, match="requires --debug"):
        _args(debug_log_path="debug.jsonl").validate()
    _args(gripper_sender_ip="10.0.0.2").validate()
    with pytest.raises(ValueError, match="requires --gripper-sender-ip"):
        _args(enable_remote_button=True).validate()
    _args(gripper_sender_ip="10.0.0.2", enable_remote_button=True).validate()


def test_run_integrates_background_tracking_backend_and_active_loop(tmp_path):
    import time

    class CyclingSource:
        def __init__(self):
            self.count = 0
            self.closed = False

        def next_pose(self):
            self.count += 1
            side = "left" if self.count % 2 else "right"
            time.sleep(0.0001)
            return TrackerPose(
                name=side,
                serial=f"{side}-tracker",
                timestamp=float(self.count),
                position=(0.0, 0.0, 0.0),
                quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            )

        def close(self):
            self.closed = True

    class ContextKeys(FakeKeys):
        def __enter__(self):
            return self

        def __exit__(self, *_exc_info):
            pass

    source = CyclingSource()
    backend = FakeBackend()
    run(
        _args("active"),
        backend_factory=lambda _args: backend,
        source_factory=lambda _args: source,
        keyboard_factory=lambda: ContextKeys([("e",), ("q",)]),
        emit=lambda _line: None,
    )

    assert len(backend.commands) == 1
    assert backend.move_calls == [DEFAULT_START_POSITION, ZERO_POSITION]
    assert backend.closed is True
    assert source.closed is True

    class FailingKeys(ContextKeys):
        def poll(self):
            raise RuntimeError("terminal failed")

    failing_source = CyclingSource()
    failing_backend = FakeBackend()
    with pytest.raises(RuntimeError, match="terminal failed"):
        run(
            _args("active"),
            backend_factory=lambda _args: failing_backend,
            source_factory=lambda _args: failing_source,
            keyboard_factory=lambda: FailingKeys([]),
            emit=lambda _line: None,
        )
    assert failing_backend.move_calls == [DEFAULT_START_POSITION, ZERO_POSITION]
    assert failing_backend.closed is True
    assert failing_source.closed is True

    dry_source = CyclingSource()
    dry_backend = FakeBackend()
    run(
        _args("dry-run"),
        backend_factory=lambda _args: dry_backend,
        source_factory=lambda _args: dry_source,
        keyboard_factory=lambda: ContextKeys([("q",)]),
        emit=lambda _line: None,
    )
    assert dry_backend.move_calls == []
    assert dry_backend.commands == []

    debug_path = tmp_path / "teleop-debug.jsonl"
    debug_source = CyclingSource()
    debug_backend = FakeBackend()
    run(
        _args("active", debug=True, debug_log_path=debug_path),
        backend_factory=lambda _args: debug_backend,
        source_factory=lambda _args: debug_source,
        keyboard_factory=lambda: ContextKeys([("e",), ("q",)]),
        emit=lambda _line: None,
    )
    records = [json.loads(line) for line in debug_path.read_text().splitlines()]
    events = [record["event"] for record in records]
    assert debug_path.stat().st_mode & 0o777 == 0o600
    assert "run_start" in events
    assert "engage" in events
    assert "control_update" in events
    assert "command_sent" in events
    assert events[-1] == "run_end"
    update = next(record for record in records if record["event"] == "control_update")
    assert set(update["tracker_samples"]) == {"left", "right"}
    assert set(update["robot_states"]) == {"left", "right"}
    assert set(update["requested_targets"]) == {"left", "right"}
    assert set(update["command_targets"]) == {"left", "right"}
    assert set(update["ik_solutions"]) == {"left", "right"}
    assert set(update["joint_commands"]) == {"left", "right"}
    assert set(update["joint_corrections"]) == {"left", "right"}
    return_begin = next(record for record in records if record["event"] == "return_to_zero_begin")
    assert return_begin["position"] == list(ZERO_POSITION)
    assert "return_to_zero_complete" in events

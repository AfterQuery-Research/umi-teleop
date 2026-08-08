from dataclasses import dataclass

import numpy as np
import pytest

from umi_teleop_fixed.robot import DEFAULT_START_POSITION, ArmState
from umi_teleop_fixed.teleop import Args, TeleopController, control_loop, run
from umi_teleop_fixed.trackers import TrackerPose, TrackerSample


def _args(mode="dry-run", **overrides):
    values = dict(
        robot_id="robot",
        left_adapter_serial="LEFT-CAN",
        right_adapter_serial="RIGHT-CAN",
        left_tracker_serial="LEFT-TRACKER",
        right_tracker_serial="RIGHT-TRACKER",
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
    return {
        side: ArmState(np.zeros(6), 0.4, np.eye(4), 1.0)
        for side in ("left", "right")
    }


def test_controller_captures_a_no_jump_baseline_and_rate_limits():
    controller = TeleopController(_args(control_rate_hz=10.0, max_linear_speed_m_s=0.2))
    controller.engage(_states(), _samples())

    initial = controller.propose(_samples(), dt=0.1)
    moved = controller.propose(_samples(x=0.1), dt=0.1)

    assert initial["left"] == pytest.approx(np.eye(4))
    assert moved["left"][0, 3] == pytest.approx(0.02)
    controller.commit(moved, controller.seeds())
    assert "left dxyz_mm=[+20.0,+0.0,+0.0]" in controller.motion_summary()


def test_controller_rejects_target_outside_engage_workspace():
    controller = TeleopController(_args(max_translation_from_start_m=0.3))
    controller.engage(_states(), _samples())

    with pytest.raises(RuntimeError, match="safe workspace"):
        controller.propose(_samples(x=0.31), dt=1 / 60)


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


def test_stale_tracker_disengages_both_arms():
    backend = FakeBackend()
    output = []
    control_loop(
        _args("active"),
        backend=backend,
        trackers=FakeTrackers(fail_after=2),
        keys=FakeKeys([("e",), (), ("q",)]),
        monotonic=lambda: 0.0,
        sleep=lambda _delay: None,
        emit=output.append,
    )

    assert backend.commands == []
    assert backend.idle_count == 3
    assert any("SAFETY STOP" in line and "stale" in line for line in output)


def test_args_reject_duplicate_hardware_ids_and_nonpositive_limits():
    with pytest.raises(ValueError, match="CAN adapter"):
        _args(right_adapter_serial="left-can").validate()
    with pytest.raises(ValueError, match="tracker serials"):
        _args(right_tracker_serial="left-tracker").validate()
    with pytest.raises(ValueError, match="control_rate_hz"):
        _args(control_rate_hz=0).validate()
    with pytest.raises(ValueError, match="14 finite"):
        _args(start_position=(0.0,) * 13).validate()


def test_run_integrates_background_tracking_backend_and_active_loop():
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
    assert backend.move_calls == [DEFAULT_START_POSITION, DEFAULT_START_POSITION]
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
    assert failing_backend.move_calls == [DEFAULT_START_POSITION, DEFAULT_START_POSITION]
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

import math

import numpy as np
import pytest
from test_lighthouses import lighthouse_config

from umi_teleop_fixed import calibrate_tracking
from umi_teleop_fixed.lighthouses import (
    load_lighthouse_calibration_id,
    require_lighthouse_calibration,
)
from umi_teleop_fixed.se3 import rpy_degrees_to_matrix
from umi_teleop_fixed.teleop_frame import load_teleop_frame
from umi_teleop_fixed.trackers import TrackerPose


def _captures(rotation, distance=0.15):
    center = np.asarray((0.4, -0.2, 1.0))
    targets = {
        "forward": (1.0, 0.0, 0.0),
        "backward": (-1.0, 0.0, 0.0),
        "left": (0.0, 1.0, 0.0),
        "right": (0.0, -1.0, 0.0),
        "up": (0.0, 0.0, 1.0),
        "down": (0.0, 0.0, -1.0),
    }
    captures = {
        name: center + rotation.T @ np.asarray(target) * distance
        for name, target in targets.items()
    }
    return center, captures


def _trajectory(center, captures):
    positions = [center] * 20
    for name in ("forward", "backward", "left", "right", "up"):
        positions.extend([captures[name]] * 20)
        positions.extend([center] * 20)
    return positions


def test_solve_frame_recovers_a_full_3d_rotation():
    expected = rpy_degrees_to_matrix((18.0, -12.0, -143.0))
    center, captures = _captures(expected)
    captures.pop("down")

    solution = calibrate_tracking.solve_frame(
        center,
        captures,
        max_axis_error_degrees=10.0,
    )

    assert solution.rotation == pytest.approx(expected, abs=1e-9)
    assert solution.direction_errors_degrees == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)


def test_solve_frame_rejects_movements_that_are_not_opposites():
    center, captures = _captures(np.eye(3))
    captures["backward"] = center + np.asarray((0.0, -0.15, 0.0))

    with pytest.raises(RuntimeError, match="opposite movements"):
        calibrate_tracking.solve_frame(
            center,
            captures,
            max_axis_error_degrees=20.0,
        )


def test_solve_frame_accepts_opposite_gestures_when_the_fitted_axis_is_accurate():
    center, captures = _captures(np.eye(3))
    angle = math.radians(16.6)
    captures["left"] = center + 0.15 * np.asarray((math.sin(angle), math.cos(angle), 0.0))
    captures["right"] = center + 0.15 * np.asarray((math.sin(angle), -math.cos(angle), 0.0))

    solution = calibrate_tracking.solve_frame(
        center,
        captures,
        max_axis_error_degrees=20.0,
    )

    assert solution.opposition_errors_degrees[1] == pytest.approx(33.2)
    assert max(solution.direction_errors_degrees) == pytest.approx(16.6)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeSource:
    def __init__(self, *, clock, positions):
        self.clock = clock
        self.positions = iter(positions)
        self.calls = 0
        self.closed = False

    def next_pose(self):
        self.clock.now += 0.05
        self.calls += 1
        try:
            position = next(self.positions)
        except StopIteration as error:
            raise AssertionError("automatic calibration consumed too many poses") from error
        return TrackerPose(
            name="WM0",
            serial="TRACKER",
            timestamp=self.clock.now,
            position=tuple(float(value) for value in position),
            quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        )

    def close(self):
        self.closed = True


def test_combined_run_automatically_advances_and_saves_a_paired_frame(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    config_path = tmp_path / "config.json"
    clock = Clock()
    center, captures = _captures(np.eye(3), distance=0.12)
    positions = _trajectory(center, captures)
    sources = []

    def lighthouse_runner(args, **_kwargs):
        args.resolved_config_path().write_text(lighthouse_config())
        return require_lighthouse_calibration(args.resolved_config_path())

    def source_factory(_args):
        source = FakeSource(clock=clock, positions=positions)
        sources.append(source)
        return source

    messages = []
    result = calibrate_tracking.run(
        calibrate_tracking.Args(
            robot_id="robot-01",
            tracker_serial="TRACKER",
            config_path=config_path,
            gesture_timeout_seconds=5.0,
            minimum_motion_m=0.08,
            stable_seconds=0.2,
            stable_radius_m=0.001,
            center_return_radius_m=0.02,
            min_tracker_rate_hz=1.0,
        ),
        lighthouse_runner=lighthouse_runner,
        pose_source_factory=source_factory,
        monotonic=clock,
        emit=messages.append,
    )

    calibration_id = load_lighthouse_calibration_id(config_path)
    saved = load_teleop_frame(
        result.frame_path,
        expected_robot_id="robot-01",
        expected_lighthouse_calibration_id=calibration_id,
    )
    assert saved.rpy_degrees == pytest.approx((0.0, 0.0, 0.0), abs=1e-7)
    assert saved.direction_errors_degrees == pytest.approx((0.0, 0.0, 0.0), abs=1e-6)
    assert sum("Distance reached" in message for message in messages) == 5
    assert sum("CENTER reached" in message for message in messages) == 5
    assert sum("Advancing to the next instruction" in message for message in messages) == 5
    assert "do not measure distance or count time" in messages[0]
    assert any("Stand behind the robot" in message for message in messages)
    assert any("Move the tracker to YOUR LEFT" in message for message in messages)
    assert messages[-1].startswith("Calibration complete")
    assert sources[0].closed is True


def test_combined_run_retries_directions_without_repeating_lighthouse_solve(
    monkeypatch,
    tmp_path,
):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    config_path = tmp_path / "config.json"
    clock = Clock()
    center, good = _captures(np.eye(3), distance=0.12)
    bad = dict(good)
    bad["backward"] = center + np.asarray((0.0, -0.12, 0.0))
    positions = _trajectory(center, bad) + _trajectory(center, good)
    lighthouse_calls = 0

    def lighthouse_runner(args, **_kwargs):
        nonlocal lighthouse_calls
        lighthouse_calls += 1
        args.resolved_config_path().write_text(lighthouse_config())
        return require_lighthouse_calibration(args.resolved_config_path())

    messages = []
    calibrate_tracking.run(
        calibrate_tracking.Args(
            robot_id="robot-01",
            tracker_serial="TRACKER",
            config_path=config_path,
            gesture_timeout_seconds=5.0,
            minimum_motion_m=0.08,
            stable_seconds=0.2,
            stable_radius_m=0.001,
            center_return_radius_m=0.02,
            min_tracker_rate_hz=1.0,
        ),
        lighthouse_runner=lighthouse_runner,
        pose_source_factory=lambda _args: FakeSource(clock=clock, positions=positions),
        monotonic=clock,
        emit=messages.append,
    )

    assert lighthouse_calls == 1
    assert any("Repeating the guided directions (attempt 2/3)" in value for value in messages)


def test_args_keep_stability_and_center_thresholds_separate():
    with pytest.raises(ValueError, match="stable_radius_m must be smaller"):
        calibrate_tracking.Args(
            robot_id="robot",
            tracker_serial="TRACKER",
            stable_radius_m=0.04,
            center_return_radius_m=0.03,
        ).validate()

import json

import pytest

from umi_teleop_fixed.teleop_frame import (
    Args,
    TeleopFrameCalibration,
    load_teleop_frame,
    run,
    save_teleop_frame,
    teleop_frame_path,
)


def _calibration(rpy=(0.0, 0.0, -150.0), calibration_id=None):
    return TeleopFrameCalibration(
        rpy_degrees=rpy,
        calibrated_at_utc="2026-08-08T23:40:00+00:00",
        source="directional experiment",
        lighthouse_calibration_id=calibration_id,
        direction_errors_degrees=(1.0, 2.0, 3.0),
    )


def test_frame_round_trip_is_private_and_robot_specific(tmp_path):
    path = teleop_frame_path("robot-01", root=tmp_path)
    save_teleop_frame(path, robot_id="robot-01", calibration=_calibration())

    loaded = load_teleop_frame(path, expected_robot_id="robot-01")

    assert loaded.rpy_degrees == (0.0, 0.0, -150.0)
    assert json.loads(path.read_text())["schema_version"] == 2
    assert path.stat().st_mode & 0o777 == 0o600
    with pytest.raises(ValueError, match="robot_id"):
        load_teleop_frame(path, expected_robot_id="other-robot")


def test_missing_frame_explains_how_to_create_it(tmp_path):
    path = teleop_frame_path("robot-01", root=tmp_path)

    with pytest.raises(RuntimeError, match="umi-calibrate-tracking"):
        load_teleop_frame(path, expected_robot_id="robot-01")


def test_frame_rejects_nonfinite_rotation():
    with pytest.raises(ValueError, match="three finite"):
        _calibration((0.0, 0.0, float("nan")))


def test_cli_run_uses_the_user_cache(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path))

    path = run(Args(robot_id="robot-01", rpy_degrees=(0.0, 0.0, -150.0)))

    assert path == teleop_frame_path("robot-01", root=tmp_path)
    assert load_teleop_frame(path, expected_robot_id="robot-01").rpy_degrees[-1] == -150.0


def test_frame_must_match_the_current_lighthouse_generation(tmp_path):
    path = teleop_frame_path("robot-01", root=tmp_path)
    save_teleop_frame(path, robot_id="robot-01", calibration=_calibration(calibration_id="a" * 64))

    with pytest.raises(RuntimeError, match="does not match"):
        load_teleop_frame(
            path,
            expected_robot_id="robot-01",
            expected_lighthouse_calibration_id="b" * 64,
        )

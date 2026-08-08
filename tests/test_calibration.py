import json
from datetime import UTC, datetime

import numpy as np
import pytest

from umi_teleop_fixed.calibration import (
    GripperCalibration,
    calibration_path,
    load_gripper_calibrations,
    save_gripper_calibration,
)


def _record(serial: str, limits: tuple[float, float]) -> GripperCalibration:
    return GripperCalibration(
        adapter_serial=serial,
        gripper_type="linear_4310",
        gripper_limits=limits,
        calibrated_at_utc="2026-08-08T12:00:00Z",
    )


def test_default_path_uses_xdg_cache_home(monkeypatch, tmp_path):
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path))

    assert calibration_path("robot-01") == (
        tmp_path / "umi-teleop-fixed/calibration/grippers/robot-01.json"
    )


def test_cache_updates_one_side_without_losing_the_other(tmp_path):
    path = calibration_path("robot-01", root=tmp_path)
    save_gripper_calibration(
        path,
        robot_id="robot-01",
        side="left",
        calibration=_record("LEFT", (2.0, -1.0)),
    )
    save_gripper_calibration(
        path,
        robot_id="robot-01",
        side="right",
        calibration=_record("RIGHT", (3.0, -2.0)),
    )

    loaded = load_gripper_calibrations(path, expected_robot_id="robot-01")
    assert loaded["left"].gripper_limits == (2.0, -1.0)
    assert loaded["right"].adapter_serial == "RIGHT"
    assert json.loads(path.read_text())["schema_version"] == 1
    assert path.stat().st_mode & 0o777 == 0o600


def test_calibration_accepts_i2rt_numpy_endpoints():
    calibration = _record("SERIAL", np.asarray([6.5, 1.2]))

    assert calibration.gripper_limits == (6.5, 1.2)


@pytest.mark.parametrize("limits", [(0.0, 0.0), (float("nan"), 1.0), (0.0,)])
def test_calibration_rejects_invalid_limits(limits):
    with pytest.raises(ValueError, match="gripper_limits"):
        _record("SERIAL", limits)


def test_timestamp_example_is_timezone_aware():
    assert datetime(2026, 8, 8, tzinfo=UTC).isoformat().endswith("+00:00")

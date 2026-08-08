import math

import numpy as np
import pytest

from umi_teleop_fixed.se3 import (
    pose_matrix,
    rate_limit_pose,
    rotation_distance,
    scaled_relative_target,
    workspace_error,
)


def _z_rotation(angle: float):
    return (math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2))


def test_relative_mapping_has_no_jump_at_engage():
    tracker = pose_matrix((4.0, -2.0, 1.0), _z_rotation(0.7))
    robot = pose_matrix((0.3, 0.1, 0.4), _z_rotation(-0.2))

    target = scaled_relative_target(
        tracker,
        tracker,
        robot,
        translation_scale=1.0,
        rotation_scale=1.0,
    )

    assert target == pytest.approx(robot)


def test_relative_mapping_scales_translation_and_rotation():
    tracker_start = pose_matrix((0.0, 0.0, 0.0), _z_rotation(0.0))
    tracker_now = pose_matrix((0.2, 0.0, 0.0), _z_rotation(math.pi / 2))
    robot_start = pose_matrix((1.0, 2.0, 3.0), _z_rotation(0.0))

    target = scaled_relative_target(
        tracker_start,
        tracker_now,
        robot_start,
        translation_scale=0.5,
        rotation_scale=0.5,
    )

    assert target[:3, 3] == pytest.approx((1.1, 2.0, 3.0))
    assert rotation_distance(robot_start, target) == pytest.approx(math.pi / 4)


def test_rate_limiter_bounds_one_update():
    previous = pose_matrix((0.0, 0.0, 0.0), _z_rotation(0.0))
    requested = pose_matrix((1.0, 0.0, 0.0), _z_rotation(math.pi))

    limited = rate_limit_pose(
        previous,
        requested,
        dt=0.1,
        max_linear_speed_m_s=0.2,
        max_angular_speed_rad_s=1.0,
    )

    assert np.linalg.norm(limited[:3, 3]) == pytest.approx(0.02)
    assert rotation_distance(previous, limited) == pytest.approx(0.1)


def test_workspace_error_checks_translation_and_rotation():
    baseline = np.eye(4)
    translated = pose_matrix((0.31, 0.0, 0.0), _z_rotation(0.0))
    rotated = pose_matrix((0.0, 0.0, 0.0), _z_rotation(1.6))

    assert "translation" in workspace_error(
        baseline, translated, max_translation_m=0.3, max_rotation_rad=1.57
    )
    assert "rotation" in workspace_error(
        baseline, rotated, max_translation_m=0.3, max_rotation_rad=1.57
    )
    assert (
        workspace_error(
            baseline, baseline, max_translation_m=0.3, max_rotation_rad=1.57
        )
        is None
    )


def test_pose_rejects_invalid_inputs():
    with pytest.raises(ValueError, match="finite"):
        pose_matrix((math.nan, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="zero"):
        pose_matrix((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 0.0))

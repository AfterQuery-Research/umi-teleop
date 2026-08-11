import math

import numpy as np
import pytest

from umi_teleop_fixed.se3 import (
    compose_child_pose,
    matrix_to_rpy_degrees,
    pose_matrix,
    rate_limit_pose,
    rotation_distance,
    rpy_degrees_to_matrix,
    scaled_relative_target,
    workspace_error,
    world_frame_delta,
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


def test_composed_child_pose_uses_canonical_umi_axes_and_rotates_lever_arm():
    tracker_to_tip = (0.0, -0.08370, 0.21395247527)
    tracker = pose_matrix((0.4, -0.2, 1.1), _z_rotation(math.pi / 2))
    tracker_to_tip_rotation = rpy_degrees_to_matrix((0.0, 0.0, 90.0))

    tip = compose_child_pose(tracker, tracker_to_tip, tracker_to_tip_rotation)

    assert tip[:3, 3] == pytest.approx((0.48370, -0.2, 1.31395247527), abs=1e-12)
    assert tip[:3, :3] == pytest.approx(rpy_degrees_to_matrix((0.0, 0.0, 180.0)))


def test_identity_tracker_maps_umi_x_up_y_right_z_forward():
    tip = compose_child_pose(
        np.eye(4),
        (0.0, -0.08370, 0.21395247527),
        rpy_degrees_to_matrix((0.0, 0.0, 90.0)),
    )

    assert tip[:3, 0] == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)
    assert tip[:3, 1] == pytest.approx((-1.0, 0.0, 0.0), abs=1e-12)
    assert tip[:3, 2] == pytest.approx((0.0, 0.0, 1.0), abs=1e-12)


def test_composed_child_pose_stays_fixed_when_tracker_rotates_about_tip():
    tracker_to_tip = np.array((0.0, -0.08370, 0.21395247527))
    tracker_to_tip_rotation = rpy_degrees_to_matrix((0.0, 0.0, 90.0))
    fixed_tip = np.array((0.6, -0.3, 0.9))
    tracker_rotation = pose_matrix((0.0, 0.0, 0.0), _z_rotation(math.pi / 2))[:3, :3]
    tracker_position = fixed_tip - tracker_rotation @ tracker_to_tip
    tracker = pose_matrix(tuple(tracker_position), _z_rotation(math.pi / 2))

    tip = compose_child_pose(tracker, tracker_to_tip, tracker_to_tip_rotation)

    assert tip[:3, 3] == pytest.approx(fixed_tip, abs=1e-12)


@pytest.mark.parametrize(
    "translation",
    ((1.0, 2.0), (1.0, 2.0, math.nan), (1.0, 2.0, math.inf)),
)
def test_composed_child_pose_rejects_invalid_translation(translation):
    with pytest.raises(ValueError, match="three finite values"):
        compose_child_pose(np.eye(4), translation, np.eye(3))


def test_fixed_child_orientation_cancels_from_relative_rotation():
    tracker_start = pose_matrix((0.0, 0.0, 0.0), _z_rotation(0.3))
    tracker_now = pose_matrix((0.0, 0.0, 0.0), _z_rotation(0.8))
    tracker_to_tip_rotation = rpy_degrees_to_matrix((0.0, 0.0, 90.0))
    tip_start = compose_child_pose(tracker_start, (0.0, 0.0, 0.0), tracker_to_tip_rotation)
    tip_now = compose_child_pose(tracker_now, (0.0, 0.0, 0.0), tracker_to_tip_rotation)

    tracker_delta = tracker_now[:3, :3] @ tracker_start[:3, :3].T
    tip_delta = tip_now[:3, :3] @ tip_start[:3, :3].T

    assert tip_delta == pytest.approx(tracker_delta, abs=1e-12)


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


def test_zero_rotation_scale_produces_translation_only_target():
    tracker_start = pose_matrix((0.0, 0.0, 0.0), _z_rotation(0.0))
    tracker_now = pose_matrix((0.0, 0.0, 0.1), _z_rotation(math.pi / 2))
    robot_start = pose_matrix((0.3, 0.1, 0.4), _z_rotation(-0.7))

    target = scaled_relative_target(
        tracker_start,
        tracker_now,
        robot_start,
        translation_scale=0.5,
        rotation_scale=0.0,
    )

    assert target[:3, 3] == pytest.approx(robot_start[:3, 3] + (0.0, 0.0, 0.05))
    assert target[:3, :3] == pytest.approx(robot_start[:3, :3])


def test_physical_up_does_not_depend_on_tracker_or_robot_orientation():
    tracker_start = pose_matrix((1.0, 2.0, 3.0), _z_rotation(math.pi / 2))
    tracker_up = pose_matrix((1.0, 2.0, 3.05), _z_rotation(math.pi / 2))
    robot_start = pose_matrix((0.3, 0.1, 0.4), _z_rotation(-0.7))

    delta = world_frame_delta(tracker_start, tracker_up)
    target = scaled_relative_target(
        tracker_start,
        tracker_up,
        robot_start,
        translation_scale=1.0,
        rotation_scale=1.0,
    )

    assert delta[:3, 3] == pytest.approx((0.0, 0.0, 0.05))
    assert target[:3, 3] == pytest.approx(robot_start[:3, 3] + (0.0, 0.0, 0.05))
    assert target[:3, :3] == pytest.approx(robot_start[:3, :3])


def test_fixed_basis_maps_tracker_world_axes_to_robot_base_axes():
    tracker_start = pose_matrix((0.0, 0.0, 0.0), _z_rotation(0.0))
    tracker_now = pose_matrix((0.1, 0.0, 0.0), _z_rotation(0.0))
    basis = rpy_degrees_to_matrix((0.0, 0.0, 90.0))

    delta = world_frame_delta(tracker_start, tracker_now, basis)

    assert delta[:3, 3] == pytest.approx((0.0, 0.1, 0.0), abs=1e-9)


@pytest.mark.parametrize(
    "rpy",
    ((0.0, 0.0, -150.0), (20.0, -30.0, 75.0), (-45.0, 89.0, 120.0)),
)
def test_rotation_matrix_round_trips_through_rpy(rpy):
    rotation = rpy_degrees_to_matrix(rpy)

    reconstructed = rpy_degrees_to_matrix(matrix_to_rpy_degrees(rotation))

    assert reconstructed == pytest.approx(rotation, abs=1e-9)


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

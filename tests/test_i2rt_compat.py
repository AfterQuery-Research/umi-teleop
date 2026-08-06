import numpy as np
import pytest

from umi_teleop.i2rt_compat import (
    apply_arm_motor_wrap_offsets,
    apply_gripper_motor_wrap_offset,
)


def test_encoder_wrap_fix_only_changes_arm_offsets() -> None:
    offsets = np.zeros(7)
    positions = np.array([-3.4, 3.5, 0.0, 0.0, 0.0, 0.0, 6.326962691691463])

    apply_arm_motor_wrap_offsets(offsets, positions, n_arm_joints=6)

    np.testing.assert_allclose(offsets[:2], [-2 * np.pi, 2 * np.pi])
    np.testing.assert_allclose(offsets[2:], 0.0)


def test_gripper_position_inside_calibrated_range_is_not_rebased() -> None:
    offsets = np.zeros(7)
    positions = np.array([0.0] * 6 + [6.326962691691463])

    corrected = apply_gripper_motor_wrap_offset(
        offsets,
        positions,
        np.ones(7),
        gripper_index=6,
        gripper_limits=np.array([6.3704509, 1.2239643]),
    )

    assert corrected == positions[6]
    assert offsets[6] == 0.0


def test_gripper_position_wrapped_below_range_is_rebased_by_one_turn() -> None:
    offsets = np.zeros(7)
    positions = np.array([0.0] * 6 + [-5.076104371709773])

    corrected = apply_gripper_motor_wrap_offset(
        offsets,
        positions,
        np.ones(7),
        gripper_index=6,
        gripper_limits=np.array([6.3967727, 1.2010758]),
    )

    assert corrected == pytest.approx(1.207080935469813)
    assert offsets[6] == pytest.approx(-2 * np.pi)


def test_gripper_wrap_offset_accounts_for_motor_direction() -> None:
    offsets = np.zeros(1)

    corrected = apply_gripper_motor_wrap_offset(
        offsets,
        np.array([-5.076104371709773]),
        np.array([-1.0]),
        gripper_index=0,
        gripper_limits=np.array([6.3967727, 1.2010758]),
    )

    assert corrected == pytest.approx(1.207080935469813)
    assert offsets[0] == pytest.approx(2 * np.pi)


def test_gripper_wrap_rejects_position_not_near_calibrated_range() -> None:
    offsets = np.zeros(1)

    with pytest.raises(ValueError, match="cannot be aligned"):
        apply_gripper_motor_wrap_offset(
            offsets,
            np.array([0.5]),
            np.ones(1),
            gripper_index=0,
            gripper_limits=np.array([6.3967727, 1.2010758]),
        )

    assert offsets[0] == 0.0

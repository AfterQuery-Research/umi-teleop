import numpy as np

from umi_teleop.i2rt_compat import apply_arm_motor_wrap_offsets


def test_encoder_wrap_fix_only_changes_arm_offsets() -> None:
    offsets = np.zeros(7)
    positions = np.array([-3.4, 3.5, 0.0, 0.0, 0.0, 0.0, 6.326962691691463])

    apply_arm_motor_wrap_offsets(offsets, positions, n_arm_joints=6)

    np.testing.assert_allclose(offsets[:2], [-2 * np.pi, 2 * np.pi])
    np.testing.assert_allclose(offsets[2:], 0.0)

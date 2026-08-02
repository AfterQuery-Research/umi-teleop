"""Small I2RT compatibility helpers that are testable without hardware dependencies."""

from __future__ import annotations

import logging

import numpy as np


def apply_arm_motor_wrap_offsets(
    motor_offsets: np.ndarray,
    motor_positions: np.ndarray,
    n_arm_joints: int,
) -> None:
    """Rebase wrapped arm encoders without changing calibrated gripper positions."""
    for index, position in enumerate(motor_positions[:n_arm_joints]):
        if position < -np.pi:
            logging.info("motor %d pos=%.3f, offset -2pi", index, position)
            motor_offsets[index] -= 2 * np.pi
        elif position > np.pi:
            logging.info("motor %d pos=%.3f, offset +2pi", index, position)
            motor_offsets[index] += 2 * np.pi

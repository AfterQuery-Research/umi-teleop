"""Small I2RT compatibility helpers that are testable without hardware dependencies."""

from __future__ import annotations

import logging

import numpy as np

GRIPPER_WRAP_TOLERANCE_RAD = 0.25


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


def apply_gripper_motor_wrap_offset(
    motor_offsets: np.ndarray,
    motor_positions: np.ndarray,
    motor_directions: np.ndarray,
    gripper_index: int,
    gripper_limits: np.ndarray,
    *,
    tolerance_rad: float = GRIPPER_WRAP_TOLERANCE_RAD,
) -> float:
    """Rebase a wrapped gripper encoder against its calibrated raw endpoints.

    Unlike arm joints, this gripper's valid calibrated range crosses ``+pi``, so
    a blanket +/-pi correction is unsafe. Choose the equivalent whole-turn
    position nearest the calibrated interval and reject it when it is not close
    enough to that interval.

    Returns the corrected position in robot joint space.
    """
    if not 0 <= gripper_index < len(motor_positions):
        raise ValueError(f"gripper index {gripper_index} is outside the motor position array")
    if len(motor_offsets) != len(motor_positions) or len(motor_directions) != len(motor_positions):
        raise ValueError("motor offsets, positions, and directions must have equal lengths")

    limits = np.asarray(gripper_limits, dtype=np.float64)
    if limits.shape != (2,) or not np.all(np.isfinite(limits)):
        raise ValueError("gripper limits must contain two finite raw endpoints")
    if not np.isfinite(tolerance_rad) or tolerance_rad < 0:
        raise ValueError("gripper wrap tolerance must be finite and nonnegative")

    position = float(motor_positions[gripper_index])
    direction = float(motor_directions[gripper_index])
    if not np.isfinite(position):
        raise ValueError("gripper position must be finite")
    if not np.isfinite(direction) or direction == 0:
        raise ValueError("gripper motor direction must be finite and nonzero")

    lower = float(np.min(limits))
    upper = float(np.max(limits))
    if lower == upper:
        raise ValueError("gripper endpoints must be distinct")

    period = 2 * np.pi
    interval_midpoint = (lower + upper) / 2
    nearest_turn = round((interval_midpoint - position) / period)
    turn_candidates = (nearest_turn - 1, nearest_turn, nearest_turn + 1)

    def distance_to_interval(candidate: float) -> float:
        if candidate < lower:
            return lower - candidate
        if candidate > upper:
            return candidate - upper
        return 0.0

    turns = min(
        turn_candidates,
        key=lambda candidate_turns: (
            distance_to_interval(position + candidate_turns * period),
            abs(candidate_turns),
        ),
    )
    corrected_position = position + turns * period
    distance = distance_to_interval(corrected_position)
    if distance > tolerance_rad:
        raise ValueError(
            "gripper encoder position cannot be aligned with calibrated limits: "
            f"position={position:.6f}, limits={limits.tolist()}, "
            f"nearest_equivalent={corrected_position:.6f}, distance={distance:.6f}rad"
        )

    if turns:
        # DMChain reports (raw - offset) * direction. Adjusting the offset by
        # -turns * 2pi / direction therefore changes only the reported branch,
        # not the physical encoder or calibrated command space.
        motor_offsets[gripper_index] -= turns * period / direction
        logging.info(
            "gripper motor %d pos=%.3f rebased by %+d turn(s) to %.3f; offset=%+.3f",
            gripper_index,
            position,
            turns,
            corrected_position,
            motor_offsets[gripper_index],
        )

    return corrected_position

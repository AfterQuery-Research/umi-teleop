"""Small, dependency-light SE(3) helpers for relative teleoperation."""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import NDArray

PoseMatrix = NDArray[np.float64]


def pose_matrix(
    position: tuple[float, float, float],
    quaternion_wxyz: tuple[float, float, float, float],
) -> PoseMatrix:
    """Build a homogeneous pose matrix from a normalized-or-normalizable quaternion."""
    translation = np.asarray(position, dtype=np.float64)
    quaternion = np.asarray(quaternion_wxyz, dtype=np.float64)
    if translation.shape != (3,) or quaternion.shape != (4,):
        raise ValueError("position and quaternion must have shapes (3,) and (4,)")
    if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(quaternion)):
        raise ValueError("pose values must be finite")
    norm = np.linalg.norm(quaternion)
    if norm < 1e-9:
        raise ValueError("quaternion must not be zero")
    w, x, y, z = quaternion / norm
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def _rotation_vector(rotation: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the shortest axis-angle vector for a proper rotation matrix."""
    trace = float(np.trace(rotation))
    angle = math.acos(float(np.clip((trace - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-9:
        return np.zeros(3, dtype=np.float64)
    if math.pi - angle < 1e-5:
        diagonal = np.maximum((np.diag(rotation) + 1.0) / 2.0, 0.0)
        axis = np.sqrt(diagonal)
        largest = int(np.argmax(axis))
        if axis[largest] < 1e-9:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            for index in range(3):
                if index != largest:
                    axis[index] = (
                        rotation[largest, index] + rotation[index, largest]
                    ) / (4.0 * axis[largest])
            axis /= np.linalg.norm(axis)
        return axis * angle
    axis = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ],
        dtype=np.float64,
    ) / (2.0 * math.sin(angle))
    return axis * angle


def _rotation_from_vector(vector: NDArray[np.float64]) -> NDArray[np.float64]:
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z = vector / angle
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def rotation_distance(first: PoseMatrix, second: PoseMatrix) -> float:
    """Return the shortest angular distance between two poses in radians."""
    relative = first[:3, :3].T @ second[:3, :3]
    return float(np.linalg.norm(_rotation_vector(relative)))


def scaled_relative_target(
    tracker_at_engage: PoseMatrix,
    tracker_now: PoseMatrix,
    robot_at_engage: PoseMatrix,
    *,
    translation_scale: float,
    rotation_scale: float,
) -> PoseMatrix:
    """Apply scaled tracker-local motion to the robot pose captured at engage."""
    relative = np.linalg.inv(tracker_at_engage) @ tracker_now
    scaled = np.eye(4, dtype=np.float64)
    scaled[:3, 3] = relative[:3, 3] * translation_scale
    scaled[:3, :3] = _rotation_from_vector(
        _rotation_vector(relative[:3, :3]) * rotation_scale
    )
    return robot_at_engage @ scaled


def rate_limit_pose(
    previous: PoseMatrix,
    requested: PoseMatrix,
    *,
    dt: float,
    max_linear_speed_m_s: float,
    max_angular_speed_rad_s: float,
) -> PoseMatrix:
    """Limit one Cartesian update without changing its destination."""
    if dt <= 0:
        raise ValueError("dt must be positive")
    result = previous.copy()
    translation_delta = requested[:3, 3] - previous[:3, 3]
    translation_distance = float(np.linalg.norm(translation_delta))
    max_translation = max_linear_speed_m_s * dt
    if translation_distance > max_translation:
        translation_delta *= max_translation / translation_distance
    result[:3, 3] += translation_delta

    relative_rotation = previous[:3, :3].T @ requested[:3, :3]
    rotation_vector = _rotation_vector(relative_rotation)
    angle = float(np.linalg.norm(rotation_vector))
    max_angle = max_angular_speed_rad_s * dt
    if angle > max_angle:
        rotation_vector *= max_angle / angle
    result[:3, :3] = previous[:3, :3] @ _rotation_from_vector(rotation_vector)
    return result


def workspace_error(
    baseline: PoseMatrix,
    target: PoseMatrix,
    *,
    max_translation_m: float,
    max_rotation_rad: float,
) -> str | None:
    """Describe a displacement outside the engage-relative workspace."""
    translation = float(np.linalg.norm(target[:3, 3] - baseline[:3, 3]))
    rotation = rotation_distance(baseline, target)
    if translation > max_translation_m:
        return f"translation {translation:.3f}m exceeds {max_translation_m:.3f}m"
    if rotation > max_rotation_rad:
        return f"rotation {rotation:.3f}rad exceeds {max_rotation_rad:.3f}rad"
    return None

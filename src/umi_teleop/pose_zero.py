"""Quaternion helpers used by the Vive coordinate conversion."""

from __future__ import annotations

import numpy as np


def _normalize(quaternion: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quaternion))
    return quaternion / norm if norm else np.array([0.0, 0.0, 0.0, 1.0])


def _conjugate(quaternion: np.ndarray) -> np.ndarray:
    return np.array([-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]])


def _multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ]
    )


def _rotate(quaternion: np.ndarray, vector: np.ndarray) -> np.ndarray:
    vector_quaternion = np.array([vector[0], vector[1], vector[2], 0.0])
    return _multiply(
        _multiply(quaternion, vector_quaternion),
        _conjugate(quaternion),
    )[:3]

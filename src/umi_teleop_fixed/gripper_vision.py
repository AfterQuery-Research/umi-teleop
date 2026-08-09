from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class FisheyeIntrinsics:
    camera_matrix: npt.NDArray[np.float64]
    distortion: npt.NDArray[np.float64]
    image_size: tuple[int, int]


@dataclass(frozen=True)
class VisionMeasurement:
    width_mm: float | None
    visible_tag_ids: tuple[int, ...]

    @property
    def valid(self) -> bool:
        return self.width_mm is not None


def load_fisheye_intrinsics(path: Path) -> FisheyeIntrinsics:
    with np.load(path) as data:
        if "K" not in data:
            raise ValueError(f"{path} must contain a K array")
        distortion_key = "D" if "D" in data else "dist"
        if distortion_key not in data:
            raise ValueError(f"{path} must contain a D or dist array")
        size_key = "image_size" if "image_size" in data else "DIM"
        if size_key not in data:
            raise ValueError(f"{path} must contain image_size or DIM")
        camera_matrix = np.asarray(data["K"], dtype=np.float64)
        distortion = np.asarray(data[distortion_key], dtype=np.float64).reshape(-1)
        raw_size = np.asarray(data[size_key]).reshape(-1)

    if camera_matrix.shape != (3, 3):
        raise ValueError(f"{path} K must be 3x3")
    if distortion.size != 4:
        raise ValueError(f"{path} D must contain four fisheye coefficients")
    if raw_size.size != 2:
        raise ValueError(f"{path} image_size must contain width and height")
    image_size = int(raw_size[0]), int(raw_size[1])
    if image_size[0] <= 0 or image_size[1] <= 0:
        raise ValueError(f"{path} image_size must be positive")
    return FisheyeIntrinsics(camera_matrix, distortion, image_size)


def _scaled_camera_matrix(
    intrinsics: FisheyeIntrinsics, frame_size: tuple[int, int]
) -> npt.NDArray[np.float64]:
    frame_width, frame_height = frame_size
    calibration_width, calibration_height = intrinsics.image_size
    scaled = intrinsics.camera_matrix.copy()
    scaled[0, :] *= frame_width / calibration_width
    scaled[1, :] *= frame_height / calibration_height
    scaled[2, :] = intrinsics.camera_matrix[2, :]
    return scaled


def width_from_fisheye_pixels(
    first_pixel: tuple[float, float],
    second_pixel: tuple[float, float],
    *,
    intrinsics: FisheyeIntrinsics,
    frame_size: tuple[int, int],
    tag_plane_offset_mm: float,
    tag_front_offset_mm: float,
) -> float:
    pixels = np.asarray([first_pixel, second_pixel], dtype=np.float64).reshape(-1, 1, 2)
    normalized = cv2.fisheye.undistortPoints(
        pixels,
        _scaled_camera_matrix(intrinsics, frame_size),
        intrinsics.distortion,
    ).reshape(-1, 2)
    rays = np.column_stack((normalized, np.ones(2, dtype=np.float64)))
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)
    angle = math.acos(float(np.clip(np.dot(rays[0], rays[1]), -1.0, 1.0)))
    camera_to_tag_line_mm = math.hypot(tag_plane_offset_mm, tag_front_offset_mm)
    return 2.0 * camera_to_tag_line_mm * math.tan(angle / 2.0)


def detect_gripper_width(
    frame: npt.NDArray[np.uint8],
    *,
    intrinsics: FisheyeIntrinsics,
    tag_ids: tuple[int, int],
    detector: cv2.aruco.ArucoDetector,
    roi_top_fraction: float,
    tag_plane_offset_mm: float,
    tag_front_offset_mm: float,
) -> VisionMeasurement:
    height, width = frame.shape[:2]
    roi_top = int(height * roi_top_fraction)
    gray = cv2.cvtColor(frame[roi_top:, :], cv2.COLOR_BGR2GRAY)
    corners, detected_ids, _ = detector.detectMarkers(gray)
    if detected_ids is None:
        return VisionMeasurement(width_mm=None, visible_tag_ids=())

    flat_ids = [int(value) for value in detected_ids.reshape(-1)]
    visible = tuple(sorted(set(flat_ids)))
    centers: dict[int, tuple[float, float]] = {}
    for marker_id, marker_corners in zip(flat_ids, corners, strict=True):
        center = np.asarray(marker_corners, dtype=np.float64).reshape(-1, 2).mean(axis=0)
        centers[marker_id] = float(center[0]), float(center[1] + roi_top)

    if tag_ids[0] not in centers or tag_ids[1] not in centers:
        return VisionMeasurement(width_mm=None, visible_tag_ids=visible)

    measured_width = width_from_fisheye_pixels(
        centers[tag_ids[0]],
        centers[tag_ids[1]],
        intrinsics=intrinsics,
        frame_size=(width, height),
        tag_plane_offset_mm=tag_plane_offset_mm,
        tag_front_offset_mm=tag_front_offset_mm,
    )
    return VisionMeasurement(width_mm=measured_width, visible_tag_ids=visible)


def create_aruco_detector() -> cv2.aruco.ArucoDetector:
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.ArucoDetector(dictionary, parameters)

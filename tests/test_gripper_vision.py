import math

import cv2
import numpy as np
import pytest

from umi_teleop_fixed.gripper_vision import (
    FisheyeIntrinsics,
    create_aruco_detector,
    detect_gripper_width,
    load_fisheye_intrinsics,
    width_from_fisheye_pixels,
)


def _intrinsics(width: int, height: int) -> FisheyeIntrinsics:
    return FisheyeIntrinsics(
        camera_matrix=np.array(
            [[400.0, 0.0, width / 2], [0.0, 400.0, height / 2], [0.0, 0.0, 1.0]]
        ),
        distortion=np.zeros(4),
        image_size=(width, height),
    )


def test_width_from_fisheye_pixels_matches_ray_geometry() -> None:
    intrinsics = _intrinsics(800, 600)
    measured = width_from_fisheye_pixels(
        (360.0, 300.0),
        (440.0, 300.0),
        intrinsics=intrinsics,
        frame_size=(800, 600),
        tag_plane_offset_mm=100.0,
        tag_front_offset_mm=0.0,
    )
    # With zero fisheye distortion, normalized pixel radius is the ray angle.
    expected_angle = 0.2
    assert measured == pytest.approx(200.0 * math.tan(expected_angle / 2.0))


def test_detector_uses_only_the_requested_tag_pair() -> None:
    width, height = 800, 600
    frame = np.full((height, width, 3), 255, dtype=np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for marker_id, x in ((2, 200), (3, 500), (0, 350)):
        marker = cv2.aruco.generateImageMarker(dictionary, marker_id, 100)
        frame[430:530, x : x + 100] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    measurement = detect_gripper_width(
        frame,
        intrinsics=_intrinsics(width, height),
        tag_ids=(2, 3),
        detector=create_aruco_detector(),
        roi_top_fraction=0.6,
        tag_plane_offset_mm=61.9,
        tag_front_offset_mm=37.1,
    )

    assert measurement.valid
    assert measurement.width_mm is not None and measurement.width_mm > 0
    assert measurement.visible_tag_ids == (0, 2, 3)


@pytest.mark.parametrize("distortion_key", ["D", "dist"])
def test_loads_both_supported_distortion_names(tmp_path, distortion_key: str) -> None:
    path = tmp_path / "camera.npz"
    np.savez(
        path,
        K=np.eye(3),
        **{distortion_key: np.zeros((4, 1))},
        image_size=np.array([1600, 1200]),
    )

    loaded = load_fisheye_intrinsics(path)

    assert loaded.image_size == (1600, 1200)
    assert loaded.distortion.shape == (4,)

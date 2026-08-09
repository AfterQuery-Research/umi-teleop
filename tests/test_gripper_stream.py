from pathlib import Path

import pytest

from umi_teleop_fixed.gripper_stream import Args, validate_args


def _args(tmp_path: Path) -> Args:
    left = tmp_path / "left.npz"
    right = tmp_path / "right.npz"
    left.touch()
    right.touch()
    return Args(
        target_host="aq-yam",
        left_camera="/dev/left",
        left_intrinsics=left,
        right_camera="/dev/right",
        right_intrinsics=right,
    )


def test_default_protocol_configuration_is_valid(tmp_path: Path) -> None:
    args = _args(tmp_path)
    validate_args(args)
    assert args.left_tag_ids == (2, 3)
    assert args.right_tag_ids == (0, 1)
    assert args.target_port == 9130


def test_rejects_shared_camera_device(tmp_path: Path) -> None:
    args = _args(tmp_path)
    invalid = Args(
        target_host=args.target_host,
        left_camera=args.left_camera,
        left_intrinsics=args.left_intrinsics,
        right_camera=args.left_camera,
        right_intrinsics=args.right_intrinsics,
    )
    with pytest.raises(ValueError, match="different devices"):
        validate_args(invalid)

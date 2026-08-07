import numpy as np
import pytest

from umi_teleop.vive import (
    R_BODY,
    MultiViveZeroer,
    euler_deg,
    libsurvive_pose_to_ros,
    openvr_pose_to_ros,
    parse_trackers,
    pose_from_packet,
    tracker_updates,
    valid_tracker_serials,
)

IDENT = np.array([0.0, 0.0, 0.0, 1.0])  # identity quaternion (x, y, z, w)


def test_euler_deg_is_gimbal_lock_stable_near_pitch_90():
    s = float(np.sin(np.pi / 4))
    q = np.array([0.0, s, 0.0, s])  # +90 deg about Y -> pitch +90 (ZYX gimbal lock)
    roll, pitch, _yaw = euler_deg(q)
    assert abs(pitch - 90.0) < 1e-3  # pitch pinned at +90
    assert roll == 0.0  # roll folded to 0 (not left to swing)
    # a tiny quaternion perturbation must NOT make roll jump (the whole point)
    qn = q + np.array([1e-4, 0.0, 1e-4, 0.0])
    qn /= np.linalg.norm(qn)
    roll_n, pitch_n, _ = euler_deg(qn)
    assert roll_n == 0.0
    assert abs(pitch_n - 90.0) < 1e-2


def test_euler_deg_matches_naive_away_from_singularity():
    s = float(np.sin(np.pi / 8))  # 45 deg about Z -> yaw 45, no pitch
    roll, pitch, yaw = euler_deg(np.array([0.0, 0.0, s, float(np.cos(np.pi / 8))]))
    np.testing.assert_allclose([roll, pitch, yaw], [0.0, 0.0, 45.0], atol=1e-6)


def test_body_correction_matrix_is_180_about_x():
    np.testing.assert_allclose(R_BODY, np.diag([1.0, -1.0, -1.0]))


def test_openvr_pose_to_ros_known_pose():
    # Guard the conversion moved out of run_vive_receiver: OpenVR (pos, wxyz-identity) ->
    # ROS pos = OPENVR_TO_ROS @ pos; rotation collapses to the R_BODY 180-about-X (quat x=1).
    p, q = openvr_pose_to_ros([1.0, 2.0, 3.0], [1.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(p, [-3.0, -1.0, 2.0], atol=1e-9)
    np.testing.assert_allclose(q, [1.0, 0.0, 0.0, 0.0], atol=1e-9)


def test_libsurvive_packet_preserves_pose_age_and_explicit_invalidation():
    packet = {
        "frame": "libsurvive_tracking",
        "trackers": [
            {
                "serial": "LEFT",
                "valid": True,
                "continuity_id": 7,
                "pose_age_s": 0.075,
                "pos": [1, 2, 3],
                "quat": [1, 0, 0, 0],
            },
            {"serial": "RIGHT", "valid": False, "continuity_id": 8},
        ],
    }

    updates = tracker_updates(packet)

    assert updates["LEFT"].sample_age_s == pytest.approx(0.075)
    assert updates["LEFT"].continuity_id == 7
    assert updates["RIGHT"].pose is None
    assert updates["RIGHT"].continuity_id == 8
    assert updates["LEFT"].pose is not None
    expected = libsurvive_pose_to_ros([1, 2, 3], [1, 0, 0, 0])
    np.testing.assert_allclose(updates["LEFT"].pose[0], expected[0])
    np.testing.assert_allclose(updates["LEFT"].pose[1], expected[1])


def test_unknown_vive_packet_frame_is_rejected():
    with pytest.raises(ValueError, match="unsupported Vive packet frame"):
        tracker_updates({"frame": "mystery", "trackers": []})


@pytest.mark.parametrize("value", [-1, 1.5, True, "1"])
def test_invalid_continuity_id_is_rejected(value):
    with pytest.raises(ValueError, match="continuity_id"):
        tracker_updates(
            {
                "trackers": [
                    {
                        "serial": "A",
                        "valid": False,
                        "continuity_id": value,
                    }
                ]
            }
        )


def test_parse_trackers_returns_all_valid_and_skips_invalid():
    packet = {
        "t": 1.0,
        "trackers": [
            {"serial": "A", "valid": True, "pos": [0, 0, 0], "quat": [1, 0, 0, 0]},
            {"serial": "B", "valid": True, "pos": [1, 0, 0], "quat": [1, 0, 0, 0]},
            {"serial": "C", "valid": False, "pos": [9, 9, 9], "quat": [1, 0, 0, 0]},
        ],
    }
    poses = parse_trackers(packet)
    assert set(poses) == {"A", "B"}  # invalid C dropped
    for pos, quat in poses.values():
        assert pos.shape == (3,)
        assert quat.shape == (4,)


def test_parse_trackers_skips_malformed_keeps_good():
    # T5: a valid-but-malformed tracker (missing pos) must not crash the packet or drop good ones.
    packet = {
        "trackers": [
            {"serial": "A", "valid": True, "pos": [0, 0, 0], "quat": [1, 0, 0, 0]},
            {"serial": "B", "valid": True, "quat": [1, 0, 0, 0]},  # no "pos"
            {"valid": True, "pos": [1, 0, 0], "quat": [1, 0, 0, 0]},  # no "serial"
        ],
    }
    assert set(parse_trackers(packet)) == {"A"}  # good one survives, malformed skipped
    assert valid_tracker_serials(packet) == ["A"]
    assert pose_from_packet(packet).serial == "A"  # doesn't KeyError on the malformed B


def test_parse_trackers_raises_on_structural_error():
    # A structurally-broken packet (trackers not a list) is a real error, not a per-tracker skip.
    with pytest.raises(ValueError):
        parse_trackers({"trackers": "nope"})


def test_absolute_until_zeroed():
    z = MultiViveZeroer()
    poses = {"A": (np.array([1.0, 2.0, 3.0]), IDENT), "B": (np.array([2.0, 2.0, 3.0]), IDENT)}
    out = z.relative(poses)
    assert z.reference is None
    np.testing.assert_allclose(out["A"], (1, 2, 3, 0, 0, 0), atol=1e-6)
    np.testing.assert_allclose(out["B"], (2, 2, 3, 0, 0, 0), atol=1e-6)


def test_zeroing_reference_reads_zero_and_others_are_relative_to_it():
    z = MultiViveZeroer()
    poses = {"A": (np.array([1.0, 2.0, 3.0]), IDENT), "B": (np.array([2.0, 2.0, 3.0]), IDENT)}
    z.request_zero("A")
    out = z.relative(poses)
    assert z.reference == "A"
    np.testing.assert_allclose(out["A"], (0, 0, 0, 0, 0, 0), atol=1e-6)  # reference -> origin
    np.testing.assert_allclose(out["B"][:3], (1, 0, 0), atol=1e-6)  # B is +1 X from A


def test_second_umi_expressed_in_reference_axes():
    z = MultiViveZeroer()
    s = float(np.sin(np.pi / 4))
    ref_yawed = np.array([0.0, 0.0, s, s])  # +90 deg yaw about Z
    poses = {"A": (np.array([0.0, 0.0, 0.0]), ref_yawed), "B": (np.array([1.0, 0.0, 0.0]), IDENT)}
    z.request_zero("A")
    out = z.relative(poses)
    # B is +1 along world X; in A's +90-yaw frame that reads as -Y.
    np.testing.assert_allclose(out["B"][:3], (0.0, -1.0, 0.0), atol=1e-6)


def test_static_frame_moving_reference_leaves_stationary_umi_unchanged():
    # Captured-at-zero: F0 is fixed at the zero instant, so each UMI's readout depends only on
    # its OWN pose. Moving the reference after zeroing must NOT move a stationary UMI's readout.
    z = MultiViveZeroer()
    a0 = (np.array([1.0, 0.0, 0.0]), IDENT)
    b0 = (np.array([2.0, 0.0, 0.0]), IDENT)
    z.request_zero("A")
    r0 = z.relative({"A": a0, "B": b0})
    delta = np.array([0.5, -0.3, 0.2])
    r1 = z.relative({"A": (a0[0] + delta, IDENT), "B": b0})  # only A moves
    np.testing.assert_allclose(r1["B"], r0["B"], atol=1e-9)  # B unchanged (fixed origin)
    assert not np.allclose(r1["A"][:3], r0["A"][:3])  # A reflects its motion


def test_re_zero_switches_reference():
    z = MultiViveZeroer()
    poses = {"A": (np.array([1.0, 0.0, 0.0]), IDENT), "B": (np.array([2.0, 0.0, 0.0]), IDENT)}
    z.request_zero("A")
    z.relative(poses)
    z.request_zero("B")  # re-anchor on B
    out = z.relative(poses)
    assert z.reference == "B"
    np.testing.assert_allclose(out["B"], (0, 0, 0, 0, 0, 0), atol=1e-6)
    np.testing.assert_allclose(out["A"][:3], (-1, 0, 0), atol=1e-6)  # A is -1 X from B

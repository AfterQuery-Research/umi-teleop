import socket
from types import SimpleNamespace

import numpy as np

from umi_teleop.windows import Config, matrix_to_quaternion, tracker_packet, udp_socket


class FakeOpenVr:
    TrackingUniverseStanding = 1
    k_unMaxTrackedDeviceCount = 2
    TrackedDeviceClass_GenericTracker = 3
    Prop_SerialNumber_String = 4


class FakeVrSystem:
    def __init__(self) -> None:
        identity = [[1.0, 0.0, 0.0, 0.1], [0.0, 1.0, 0.0, 0.2], [0.0, 0.0, 1.0, 0.3]]
        self.poses = [
            SimpleNamespace(bPoseIsValid=True, mDeviceToAbsoluteTracking=identity),
            SimpleNamespace(bPoseIsValid=False),
        ]

    def getDeviceToAbsoluteTrackingPose(self, *_args):
        return self.poses

    def getTrackedDeviceClass(self, _index: int) -> int:
        return FakeOpenVr.TrackedDeviceClass_GenericTracker

    def getStringTrackedDeviceProperty(self, index: int, _property: int) -> str:
        return f"TRACKER-{index}"


def test_identity_matrix_produces_identity_quaternion() -> None:
    matrix = [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]]
    np.testing.assert_allclose(matrix_to_quaternion(matrix), [1.0, 0.0, 0.0, 0.0])


def test_tracker_packet_contains_valid_and_invalid_trackers() -> None:
    packet = tracker_packet(FakeVrSystem(), FakeOpenVr, timestamp=123.0)
    assert packet["t"] == 123.0
    assert packet["trackers"][0] == {
        "serial": "TRACKER-0",
        "valid": True,
        "pos": [0.1, 0.2, 0.3],
        "quat": [1.0, 0.0, 0.0, 0.0],
    }
    assert packet["trackers"][1] == {"serial": "TRACKER-1", "valid": False}


def test_sender_defaults_to_local_network_broadcast() -> None:
    assert Config().target_host == "255.255.255.255"


def test_udp_socket_enables_broadcast() -> None:
    sock = udp_socket()
    try:
        assert sock.getsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST) != 0
    finally:
        sock.close()

from __future__ import annotations

import json
import socket
import time
from types import SimpleNamespace

import numpy as np

from umi_teleop.controller import GripperMap, TeleopLimits, TeleopStateMachine, UdpTeleopReceiver
from umi_teleop.protocol import ButtonState, VivePacketReceiver, encode_teleop_packet
from umi_teleop.types import Event
from umi_teleop.windows import tracker_packet


class FakeOpenVr:
    TrackingUniverseStanding = 1
    k_unMaxTrackedDeviceCount = 2
    TrackedDeviceClass_GenericTracker = 3
    Prop_SerialNumber_String = 4


class FakeVrSystem:
    def __init__(self) -> None:
        self.positions = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]

    def getDeviceToAbsoluteTrackingPose(self, *_args):
        poses = []
        for x, y, z in self.positions:
            matrix = [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z]]
            poses.append(SimpleNamespace(bPoseIsValid=True, mDeviceToAbsoluteTracking=matrix))
        return poses

    def getTrackedDeviceClass(self, _index: int) -> int:
        return FakeOpenVr.TrackedDeviceClass_GenericTracker

    def getStringTrackedDeviceProperty(self, index: int, _property: int) -> str:
        return ("LEFT", "RIGHT")[index]


def wait_for_vive(receiver: VivePacketReceiver, count: int) -> dict:
    deadline = time.monotonic() + 1.0
    while receiver.packet_count < count and time.monotonic() < deadline:
        time.sleep(0.005)
    assert receiver.packet_count >= count
    return receiver.snapshot()


def send_teleop(
    sock: socket.socket,
    receiver: UdpTeleopReceiver,
    packet: dict,
) -> tuple[dict, float]:
    sock.sendto(encode_teleop_packet(packet), ("127.0.0.1", receiver.port))
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        received, age = receiver.poll()
        if received is not None and received["seq"] == packet["seq"]:
            assert age is not None
            return received, age
        time.sleep(0.005)
    raise AssertionError("YAM receiver did not receive the expected sequence")


def test_windows_to_source_to_yam_state_machine_over_udp() -> None:
    vive_receiver = VivePacketReceiver({"LEFT": "left", "RIGHT": "right"}, host="127.0.0.1", port=0)
    yam_receiver = UdpTeleopReceiver("127.0.0.1", 0)
    windows_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    source_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    vr = FakeVrSystem()
    state = ButtonState(session_id="integration")
    machine = TeleopStateMachine(
        {"left": GripperMap(40.0, 120.0), "right": GripperMap(40.0, 120.0)},
        TeleopLimits(max_translation_m=0.4, position_scale=1.5),
    )
    robot_poses = {"left": np.eye(4), "right": np.eye(4)}
    widths = {
        "left": {"width_mm": 80.0, "age_s": 0.01},
        "right": {"width_mm": 100.0, "age_s": 0.01},
    }

    vive_receiver.start()
    try:
        raw = tracker_packet(vr, FakeOpenVr, timestamp=1.0)
        windows_socket.sendto(json.dumps(raw).encode(), ("127.0.0.1", vive_receiver.port))
        poses = wait_for_vive(vive_receiver, 1)

        received, age = send_teleop(source_socket, yam_receiver, state.packet(0, poses, widths))
        assert "tap ZERO" in machine.step(received, age, robot_poses).reason

        state.handle(Event.ZERO)
        received, age = send_teleop(source_socket, yam_receiver, state.packet(1, poses, widths))
        zeroed = machine.step(received, age, robot_poses)
        assert zeroed.zeroed and zeroed.zero_captured and not zeroed.armed

        state.handle(Event.START)
        received, age = send_teleop(source_socket, yam_receiver, state.packet(2, poses, widths))
        assert machine.step(received, age, robot_poses).armed

        vr.positions[0][2] = -0.1
        raw = tracker_packet(vr, FakeOpenVr, timestamp=2.0)
        windows_socket.sendto(json.dumps(raw).encode(), ("127.0.0.1", vive_receiver.port))
        moved_poses = wait_for_vive(vive_receiver, 2)
        received, age = send_teleop(
            source_socket, yam_receiver, state.packet(3, moved_poses, widths)
        )
        result = machine.step(received, age, robot_poses)

        assert result.armed
        np.testing.assert_allclose(result.targets["left"][:3, 3], [0.15, 0.0, 0.0])
        assert result.grippers == {"left": 0.5, "right": 0.75}

        stale = machine.step(received, 2.0, robot_poses)
        assert not stale.armed and "telemetry stale" in stale.reason
    finally:
        vive_receiver.stop()
        yam_receiver.close()
        windows_socket.close()
        source_socket.close()

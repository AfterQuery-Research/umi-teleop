from datetime import UTC, datetime
from pathlib import Path

from umi_teleop_fixed import calibrate_gripper
from umi_teleop_fixed.calibration import load_gripper_calibrations
from umi_teleop_fixed.can import CANInterfaceInfo


def _ready_interface() -> CANInterfaceInfo:
    return CANInterfaceInfo(
        name="can7",
        adapter_serial="LEFT",
        driver="gs_usb",
        is_up=True,
        lower_up=True,
        operating_state="UP",
        can_state="ERROR-ACTIVE",
        bitrate=1_000_000,
        is_fd=False,
    )


def test_calibration_moves_only_selected_fake_gripper_and_saves(monkeypatch, tmp_path):
    opened_channels = []
    monkeypatch.setattr(calibrate_gripper, "discover_can_interfaces", lambda: [_ready_interface()])

    def detector(channel):
        opened_channels.append(channel)
        return 6.5, 1.2

    path = calibrate_gripper.run(
        calibrate_gripper.Args(
            side="left",
            robot_id="robot-01",
            left_adapter_serial="LEFT",
            right_adapter_serial="RIGHT",
        ),
        limit_detector=detector,
        root=tmp_path,
        countdown_seconds=0,
        now=lambda: datetime(2026, 8, 8, 12, 30, tzinfo=UTC),
    )

    assert opened_channels == ["can7"]
    loaded = load_gripper_calibrations(path, expected_robot_id="robot-01")
    assert loaded["left"].gripper_limits == (6.5, 1.2)
    assert loaded["left"].adapter_serial == "LEFT"
    assert loaded["left"].calibrated_at_utc == "2026-08-08T12:30:00Z"


def test_source_package_does_not_reference_lerobot():
    source_root = Path(__file__).parents[1] / "src/umi_teleop_fixed"
    for source_file in source_root.glob("*.py"):
        assert "lerobot" not in source_file.read_text().lower()

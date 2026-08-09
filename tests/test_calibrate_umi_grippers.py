from datetime import UTC, datetime

from umi_teleop_fixed.calibrate_umi_grippers import Args, run
from umi_teleop_fixed.calibration import load_umi_gripper_calibrations
from umi_teleop_fixed.gripper_receiver import ReceivedGripperSample


class Clock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value

    def sleep(self, duration):
        # Real schedulers do not wake at exact multiples of the requested period.
        self.value += duration * 1.013


class FakeReceiver:
    def __init__(self, clock):
        self.clock = clock
        # Deliberately unusual values verify that calibration discovers the
        # physical endpoints instead of expecting a particular closed width.
        self.widths = {"left": 137.0, "right": 96.0}

    def __enter__(self):
        return self

    def __exit__(self, *_exc_info):
        pass

    def read(self, **_kwargs):
        return (
            {
                side: ReceivedGripperSample(
                    width_mm=width,
                    visible_tag_ids=(2, 3),
                    received_at=self.clock.value,
                    camera_age_seconds=0.01,
                    sequence=int(self.clock.value * 100),
                    stream_id="stream-a",
                    sender="umi-pi",
                    source_ip="10.0.0.2",
                )
                for side, width in self.widths.items()
            },
            {},
        )

    def snapshot(self, **_kwargs):
        return self.read()[0]


def test_calibration_advances_on_stability_and_saves_both_spans(tmp_path):
    clock = Clock()
    receiver = FakeReceiver(clock)
    output = []

    def emit(line):
        output.append(line)
        if line.startswith("Fully CLOSE the LEFT"):
            receiver.widths["left"] = 73.0
        elif line.startswith("Fully OPEN the LEFT"):
            receiver.widths["left"] = 137.0
        elif line.startswith("Fully CLOSE the RIGHT"):
            receiver.widths["right"] = 18.0
        elif line.startswith("Fully OPEN the RIGHT"):
            receiver.widths["right"] = 96.0

    path = run(
        Args(
            robot_id="robot-01",
            sender_ip="10.0.0.2",
            stable_seconds=0.2,
            stable_range_mm=0.5,
            instruction_delay_seconds=0.1,
            stage_timeout_seconds=2.0,
        ),
        receiver_factory=lambda _args: receiver,
        root=tmp_path,
        monotonic=clock.now,
        sleep=clock.sleep,
        emit=emit,
        now=lambda: datetime(2026, 8, 8, 12, 0, tzinfo=UTC),
    )

    loaded = load_umi_gripper_calibrations(
        path,
        expected_robot_id="robot-01",
        expected_sender_ip="10.0.0.2",
    )
    assert loaded["left"].closed_width_mm == 73.0
    assert loaded["left"].open_width_mm == 137.0
    assert loaded["left"].span_mm == 64.0
    assert loaded["right"].closed_width_mm == 18.0
    assert loaded["right"].open_width_mm == 96.0
    assert loaded["right"].span_mm == 78.0
    instructions = [line for line in output if line.startswith("Fully")]
    assert instructions == [
        "Fully CLOSE the LEFT UMI gripper and hold it still.",
        "Fully OPEN the LEFT UMI gripper and hold it still.",
        "Fully CLOSE the RIGHT UMI gripper and hold it still.",
        "Fully OPEN the RIGHT UMI gripper and hold it still.",
    ]
    assert any(
        "Captured starting OPEN widths: left=137.00 mm, right=96.00 mm" in line
        for line in output
    )

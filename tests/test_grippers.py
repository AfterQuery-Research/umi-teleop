from umi_teleop_fixed import grippers
from umi_teleop_fixed.gripper_receiver import (
    ReceivedButtonEvents,
    ReceivedGripperSample,
    ReceiverStats,
)


def test_diagnostic_reports_widths_and_button_events(monkeypatch, capsys) -> None:
    class Receiver:
        def __enter__(self):
            return self

        def __exit__(self, *_exc_info):
            pass

        def poll_button_events(self, **_kwargs):
            return ReceivedButtonEvents(1, 0, False, "stream", "pi", "10.0.0.2")

        def snapshot(self, **_kwargs):
            return {
                side: ReceivedGripperSample(
                    width_mm=120.0,
                    visible_tag_ids=(2, 3),
                    received_at=0.0,
                    camera_age_seconds=0.0,
                    sequence=1,
                    stream_id="stream",
                    sender="pi",
                    source_ip="10.0.0.2",
                )
                for side in ("left", "right")
            }

        def stats(self):
            return ReceiverStats(1, 0, 0, 0, 0)

    times = iter((0.0, 0.0, 0.0, 0.02))
    monkeypatch.setattr(grippers, "GripperReceiver", lambda **_kwargs: Receiver())
    monkeypatch.setattr(grippers.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(grippers.time, "sleep", lambda _seconds: None)

    grippers.run(grippers.Args(sender_ip="10.0.0.2", duration_seconds=0.01))

    output = capsys.readouterr().out
    assert "GPIO: engage=1 quit=0" in output
    assert "left=120.00mm" in output
    assert "button=released" in output


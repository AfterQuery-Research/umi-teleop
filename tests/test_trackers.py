import math
import queue
import time

import pytest

from umi_teleop_fixed import trackers


def _pose(serial="SERIAL", timestamp=1.0, quaternion=(1.0, 0.0, 0.0, 0.0)):
    return trackers.TrackerPose(
        name="WM0",
        serial=serial,
        timestamp=timestamp,
        position=(1.0, 2.0, 3.0),
        quaternion_wxyz=quaternion,
    )


def test_pose_normalizes_quaternion_and_rejects_invalid_values():
    pose = _pose(quaternion=(2.0, 0.0, 0.0, 0.0))
    assert pose.quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)

    with pytest.raises(ValueError, match="non-finite"):
        _pose(timestamp=math.nan)
    with pytest.raises(ValueError, match="invalid quaternion"):
        _pose(quaternion=(0.0, 0.0, 0.0, 0.0))


def test_monitor_assigns_roles_and_ignores_unselected_trackers():
    monitor = trackers.TrackerMonitor(left_serial="LEFT", right_serial="RIGHT")

    assert monitor.observe(_pose("OTHER"), received_at=0.0) is False
    assert monitor.observe(_pose("left", 1.0), received_at=0.0) is True
    assert monitor.observe(_pose("LEFT", 2.0), received_at=0.01) is True
    assert monitor.observe(_pose("RIGHT", 1.0), received_at=0.0) is True
    assert monitor.observe(_pose("RIGHT", 2.0), received_at=0.02) is True

    reports = monitor.reports(now=0.02)
    assert [report.role for report in reports] == ["left", "right"]
    assert reports[0].rate_hz == pytest.approx(100.0)
    assert reports[1].rate_hz == pytest.approx(50.0)


def test_monitor_discovers_exactly_two_healthy_trackers():
    monitor = trackers.TrackerMonitor(left_serial=None, right_serial=None)
    for serial in ("ONE", "TWO"):
        monitor.observe(_pose(serial, 1.0), received_at=0.0)
        monitor.observe(_pose(serial, 2.0), received_at=0.01)

    assert monitor.health_errors(
        now=0.01,
        expected_count=2,
        stale_after_seconds=0.25,
        min_rate_hz=20.0,
    ) == ()


def test_monitor_reports_missing_stale_and_slow_tracking():
    monitor = trackers.TrackerMonitor(left_serial="LEFT", right_serial="RIGHT")
    monitor.observe(_pose("LEFT", 1.0), received_at=0.0)
    monitor.observe(_pose("LEFT", 2.0), received_at=0.2)

    errors = monitor.health_errors(
        now=1.0,
        expected_count=2,
        stale_after_seconds=0.25,
        min_rate_hz=20.0,
    )

    assert "no poses received for right tracker" in errors
    assert any("left tracker pose is stale" in error for error in errors)
    assert any("left tracker update rate is too low" in error for error in errors)


def test_args_require_both_distinct_serials():
    with pytest.raises(ValueError, match="supplied together"):
        trackers.Args(left_tracker_serial="LEFT").validated_serials()
    with pytest.raises(ValueError, match="must be different"):
        trackers.Args(
            left_tracker_serial="same",
            right_tracker_serial="SAME",
        ).validated_serials()


def test_run_closes_source_when_tracking_raises():
    class FailingSource:
        closed = False

        def next_pose(self):
            raise RuntimeError("tracking stopped")

        def close(self):
            self.closed = True

    source = FailingSource()

    with pytest.raises(RuntimeError, match="tracking stopped"):
        trackers.run(
            trackers.Args(duration_seconds=0.1),
            source_factory=lambda _args: source,
        )

    assert source.closed is True


def test_background_reader_selects_roles_and_enforces_freshness():
    sentinel = object()

    class Source:
        def __init__(self):
            self.poses = queue.Queue()
            self.closed = False

        def next_pose(self):
            value = self.poses.get(timeout=0.1)
            return None if value is sentinel else value

        def close(self):
            self.closed = True

    source = Source()
    reader = trackers.TrackerReader(
        source,
        serials={"left": "LEFT", "right": "RIGHT"},
    )
    reader.start()
    source.poses.put(_pose("OTHER"))
    source.poses.put(_pose("LEFT"))
    source.poses.put(_pose("RIGHT"))
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        try:
            samples = reader.snapshot(now=time.monotonic(), stale_after_seconds=1.0)
            break
        except RuntimeError:
            time.sleep(0.001)
    else:
        raise AssertionError("reader did not publish both tracker poses")

    assert set(samples) == {"left", "right"}
    with pytest.raises(RuntimeError, match="stale"):
        reader.snapshot(now=time.monotonic() + 2.0, stale_after_seconds=0.1)
    source.poses.put(sentinel)
    reader.close()
    assert source.closed is True


def test_background_reader_surfaces_worker_failure():
    class Source:
        def next_pose(self):
            raise ValueError("USB failed")

        def close(self):
            pass

    reader = trackers.TrackerReader(
        Source(),
        serials={"left": "LEFT", "right": "RIGHT"},
    )
    reader.start()
    reader._thread.join(timeout=1.0)
    with pytest.raises(RuntimeError, match="USB failed"):
        reader.snapshot(now=1.0, stale_after_seconds=0.1)
    reader.close()

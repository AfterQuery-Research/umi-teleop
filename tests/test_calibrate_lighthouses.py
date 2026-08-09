from test_lighthouses import lighthouse_config

from umi_teleop_fixed import calibrate_lighthouses
from umi_teleop_fixed.trackers import TrackerPose


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeSource:
    def __init__(
        self,
        *,
        clock,
        candidate,
        solve,
        lighthouse_ids=("fresh-0", "fresh-1"),
        tracker_serials=("LEFT", "RIGHT"),
    ):
        self.clock = clock
        self.candidate = candidate
        self.solve = solve
        self.lighthouse_ids = lighthouse_ids
        self.tracker_serials = tracker_serials
        self.calls = 0
        self.closed = False

    def next_pose(self):
        self.clock.now += 0.1
        self.calls += 1
        if self.solve and self.calls == 2:
            self.candidate.write_text(lighthouse_config(ids=self.lighthouse_ids))
        serial = self.tracker_serials[(self.calls - 1) % len(self.tracker_serials)]
        return TrackerPose(
            name=serial,
            serial=serial,
            timestamp=float(self.calls),
            position=(0.0, 0.0, 0.0),
            quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        )

    def close(self):
        self.closed = True


def test_create_source_forwards_expected_lighthouse_count(monkeypatch, tmp_path):
    captured = {}

    class Source:
        def __init__(self, **kwargs):
            captured.update(kwargs)

    monkeypatch.setattr(calibrate_lighthouses, "LibsurviveSource", Source)

    calibrate_lighthouses._create_source(
        calibrate_lighthouses.Args(tracker_serial="TRACKER", expected_lighthouses=3),
        tmp_path / "candidate.json",
    )

    assert captured["lighthouse_count"] == 3
    assert captured["force_calibration"] is True


def test_calibration_atomically_installs_fresh_candidate(tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(lighthouse_config(ids=("old-0", "old-1")))
    clock = Clock()
    sources = []

    def source_factory(_args, candidate):
        source = FakeSource(clock=clock, candidate=candidate, solve=True)
        sources.append(source)
        return source

    status = calibrate_lighthouses.run(
        calibrate_lighthouses.Args(
            tracker_serial="LEFT",
            timeout_seconds=2.0,
            settle_seconds=0.3,
            min_tracker_rate_hz=1.0,
            config_path=config_path,
        ),
        source_factory=source_factory,
        monotonic=clock,
        emit=lambda _message: None,
    )

    assert status.ready(expected_count=2)
    assert "fresh-0" in config_path.read_text()
    assert sources[0].closed is True
    assert not list(tmp_path.glob(".lighthouse-calibration.*"))


def test_failed_calibration_preserves_existing_config(tmp_path):
    config_path = tmp_path / "config.json"
    original = lighthouse_config(ids=("old-0", "old-1"))
    config_path.write_text(original)
    clock = Clock()

    def source_factory(_args, candidate):
        return FakeSource(clock=clock, candidate=candidate, solve=False)

    try:
        calibrate_lighthouses.run(
            calibrate_lighthouses.Args(
                tracker_serial="LEFT",
                timeout_seconds=0.5,
                settle_seconds=0.1,
                min_tracker_rate_hz=1.0,
                config_path=config_path,
            ),
            source_factory=source_factory,
            monotonic=clock,
            emit=lambda _message: None,
        )
    except RuntimeError as error:
        assert "previous calibration was preserved" in str(error)
    else:
        raise AssertionError("calibration unexpectedly succeeded")

    assert config_path.read_text() == original
    assert not list(tmp_path.glob(".lighthouse-calibration.*"))


def test_calibration_uses_only_the_required_tracker(tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(lighthouse_config())
    clock = Clock()

    def source_factory(_args, candidate):
        return FakeSource(
            clock=clock,
            candidate=candidate,
            solve=True,
            tracker_serials=("UNRELATED",),
        )

    try:
        calibrate_lighthouses.run(
            calibrate_lighthouses.Args(
                tracker_serial="SELECTED",
                timeout_seconds=2.0,
                settle_seconds=0.3,
                min_tracker_rate_hz=1.0,
                config_path=config_path,
            ),
            source_factory=source_factory,
            monotonic=clock,
            emit=lambda _message: None,
        )
    except RuntimeError as error:
        assert "received poses from 0" in str(error)
    else:
        raise AssertionError("calibration accepted an unrelated tracker")


def test_calibration_supports_three_lighthouses(tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(lighthouse_config(ids=("old-0", "old-1", "old-2")))
    clock = Clock()

    def source_factory(_args, candidate):
        return FakeSource(
            clock=clock,
            candidate=candidate,
            solve=True,
            lighthouse_ids=("fresh-0", "fresh-1", "fresh-2"),
            tracker_serials=("TRACKER",),
        )

    status = calibrate_lighthouses.run(
        calibrate_lighthouses.Args(
            tracker_serial="TRACKER",
            expected_lighthouses=3,
            timeout_seconds=2.0,
            settle_seconds=0.3,
            min_tracker_rate_hz=1.0,
            config_path=config_path,
        ),
        source_factory=source_factory,
        monotonic=clock,
        emit=lambda _message: None,
    )

    assert status.ready(expected_count=3)
    assert "fresh-2" in config_path.read_text()


def test_calibration_args_require_tracker_and_supported_lighthouse_count():
    for args, message in (
        (calibrate_lighthouses.Args(tracker_serial=" "), "tracker_serial"),
        (
            calibrate_lighthouses.Args(tracker_serial="TRACKER", expected_lighthouses=9),
            "between 2 and 8",
        ),
    ):
        try:
            args.validate()
        except ValueError as error:
            assert message in str(error)
        else:
            raise AssertionError("invalid calibration arguments were accepted")

# umi-teleop-fixed

A standalone UMI teleoperation stack for YAM robots. It uses i2rt directly and
does not import or depend on LeRobot.

## Vive trackers

Install libsurvive, turn on both trackers and base stations, then discover and
validate two live poses:

```bash
uv run umi-trackers
```

The command prints each tracker's stable serial, raw libsurvive-frame pose,
update rate, and pose age. If libsurvive is not installed in a standard system
or user-local path, pass `--libsurvive-path /path/to/libsurvive.so`. Once the
physical left/right assignment is known, validate it explicitly:

```bash
uv run umi-trackers \
    --left-tracker-serial LEFT_TRACKER_SERIAL \
    --right-tracker-serial RIGHT_TRACKER_SERIAL
```

Tracking is read-only and never connects to the robot. Libsurvive configuration
is stored under `~/.cache/umi-teleop-fixed/libsurvive/config.json` by default.

## CAN

CAN adapters are selected by USB serial because Linux `canN` names can change
after a reboot or USB replug. Discover the connected serials without changing
link state or sending CAN frames:

```bash
uv run umi-setup-can
```

Identify which discovered USB adapter is connected to each arm, then set a
stable robot name and the two returned serials in your current shell:

```bash
ROBOT_ID=YOUR_ROBOT_ID
LEFT_CAN=LEFT_ADAPTER_SERIAL
RIGHT_CAN=RIGHT_ADAPTER_SERIAL
```

Configure and verify both classic CAN links at 1 Mbps:

```bash
uv run umi-setup-can \
    --left-adapter-serial "$LEFT_CAN" \
    --right-adapter-serial "$RIGHT_CAN"
```

Run this after a reboot or adapter replug. Do not start robot control unless
both links report `UP/LOWER_UP`, `ERROR-ACTIVE`, `1000000`, and `Classic CAN`.

## Gripper calibration

Calibration moves one gripper through its full travel. Clear the workspace and
keep hands away.

```bash
uv run umi-calibrate-gripper \
    --robot-id "$ROBOT_ID" \
    --adapter-serial "$LEFT_CAN" \
    --side left

uv run umi-calibrate-gripper \
    --robot-id "$ROBOT_ID" \
    --adapter-serial "$RIGHT_CAN" \
    --side right
```

All three arguments are required. The selected CAN adapter must be healthy.
Endpoints are cached under
`~/.cache/umi-teleop-fixed/calibration/grippers/<robot-id>.json`; normal startup
must load them and never recalibrate implicitly.

## Teleoperation

First identify which tracker is physically left and right by moving one tracker
at a time while watching `umi-trackers`. Start with the full control path in
dry-run mode; it connects both arms in gravity compensation and computes live
FK, relative targets, and IK, but sends no position commands:

```bash
uv run umi-teleop \
    --robot-id YOUR_ROBOT_ID \
    --left-adapter-serial LEFT_ADAPTER_SERIAL \
    --right-adapter-serial RIGHT_ADAPTER_SERIAL \
    --left-tracker-serial LEFT_TRACKER_SERIAL \
    --right-tracker-serial RIGHT_TRACKER_SERIAL
```

The controls are `e` to engage at the current tracker and robot poses, `d` to
return both arms to gravity compensation, `r` to re-zero, and `q` to disengage
and quit. While dry-running, move one tracker at a time and confirm that only
the corresponding `left` or `right` Cartesian delta changes. The grippers hold
their measured positions. A stale pose, failed IK, dead robot worker, or
engage-relative workspace violation disengages both arms.

After dry-run validation, enable robot position commands explicitly:

```bash
uv run umi-teleop \
    --robot-id YOUR_ROBOT_ID \
    --left-adapter-serial LEFT_ADAPTER_SERIAL \
    --right-adapter-serial RIGHT_ADAPTER_SERIAL \
    --left-tracker-serial LEFT_TRACKER_SERIAL \
    --right-tracker-serial RIGHT_TRACKER_SERIAL \
    --mode active
```

Defaults are 60 Hz, 1:1 translation and rotation, 0.2 m/s and 1.0 rad/s rate
limits, and engage-relative bounds of 0.3 m and 1.57 rad. Run
`uv run umi-teleop --help` to tune them.

## Development

```bash
uv run ruff check .
uv run pytest
```

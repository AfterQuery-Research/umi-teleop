# umi-teleop-fixed

A standalone UMI teleoperation stack for YAM robots. It uses i2rt directly and
does not import or depend on LeRobot.

## Vive trackers

Install libsurvive and turn on at least one tracker and all base stations.
Whenever a Lighthouse may have moved, run the complete interactive tracking
calibration with one tracker visible to every Lighthouse:

```bash
uv run umi-calibrate-tracking \
    --robot-id YOUR_ROBOT_ID \
    --tracker-serial TRACKER_SERIAL
```

First keep the tracker stationary while MPFIT solves the Lighthouse poses. The
command then guides forward, backward, left, right, and up relative to the
robot. Stand behind the robot and face the same direction as the robot, so your
forward, left, and right match its directions. Follow only the instruction
currently displayed: the program tells you when to keep moving, when to stop and
hold, and when to return to center. It measures distance and stability itself,
then advances automatically. No key presses, measuring, or counting are needed.
The six directions align Lighthouse-world axes with robot-base axes; the robot
never connects or moves during this process.

The command expects two Lighthouses by default. For a larger SteamVR 2.0 setup,
pass `--expected-lighthouses N` (libsurvive supports up to eight safely through
its current MPFIT calibration path). Lighthouse geometry and the directional
frame are paired, so teleoperation refuses a stale frame after a new Lighthouse
solve. The Lighthouses must not move between calibration and teleoperation.

Then discover and validate two live tracker poses:

```bash
uv run umi-trackers --duration-seconds 60
```

The command is read-only and prints each tracker's stable serial, absolute pose,
update rate, pose age, displacement from its first sample (`dxyz_mm`, `drot`),
and the distance between trackers. If it warns that lighthouse positions are
not set, translational tracking must be fixed before teleoperation. If
libsurvive is not installed in a standard system or user-local path, pass
`--libsurvive-path /path/to/libsurvive.so`. Once the physical left/right
assignment is known, validate it explicitly:

```bash
uv run umi-trackers \
    --left-tracker-serial LEFT_TRACKER_SERIAL \
    --right-tracker-serial RIGHT_TRACKER_SERIAL \
    --duration-seconds 60
```

Tracking is read-only and never connects to the robot. It requires all detected
lighthouses to have saved world poses and freezes calibration so the world
frame cannot shift. Libsurvive configuration is stored under
`~/.cache/umi-teleop-fixed/libsurvive/config.json` by default.

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
at a time while watching `umi-trackers`. The combined tracking calibration above
saves the room-to-robot rotation by `robot-id`; teleoperation loads it and checks
that it belongs to the current Lighthouse solve. For a temporary diagnostic,
`--tracker-to-robot-rpy-degrees ROLL PITCH YAW` overrides the cache without
modifying it.

Start with the full control path in dry-run mode; it connects both arms in
gravity compensation and computes live FK, relative targets, and IK, but sends
no position commands. It validates the configured start position without
executing the startup or zero-return trajectory:

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
    --start-position 0 0.4 0.8 0 0 0 1 0 0.4 0.8 0 0 0 1 \
    --mode active
```

`start-position` is ordered as six left joints, left gripper, six right joints,
and right gripper. Its default is the bent ready pose above; the all-zero arm
pose is not suitable for Cartesian control because it is singular. Active mode validates the
target, moves both arms there in measured 0.01-unit steps at 30 Hz, and only
then accepts `e`. On shutdown or Ctrl-C it disengages teleoperation and returns
both arms to the fixed all-zero joint position with both grippers open,
independent of `--start-position`, before closing. A failed trajectory
immediately returns both arms to gravity compensation and reports an error.

Defaults are 60 Hz, 1:1 translation and rotation, 0.2 m/s and 1.0 rad/s rate
limits, and engage-relative bounds of 0.3 m and 1.57 rad. Tracker motion is
computed in the lighthouse world frame, so physical up commands robot up
regardless of the UMI orientation at engage. If the room's horizontal axes do
not match the robot, update the cached teleop frame above.
For a translation-only diagnostic, pass `--rotation-scale 0`; tracker rotation
is then ignored while tracker translation continues to command the physical
LINEAR_4310 gripper's `grasp_site`.
Run
`uv run umi-teleop --help` to tune them.

Add `--debug` to write lossless per-cycle JSONL telemetry under
`~/.cache/umi-teleop-fixed/logs/`. The command prints the exact path at startup.
Each control record contains raw tracker poses, measured robot state, requested
Cartesian targets, rate-limited command targets, IK seeds and solutions,
feedback-compensated joint commands, joint corrections, and gripper targets.
Key actions, safety stops, startup/return motion, and normal or failed shutdown
are also recorded. Use `--debug-log-path PATH` with `--debug` to choose a
specific destination.

## Development

```bash
uv run ruff check .
uv run pytest
```

# UMI Teleop

Standalone Vive teleoperation for two camera-equipped UMIs and two YAM robot arms.

> **Physical safety:** `umi-yam --confirm-clear` energizes both arms and moves them automatically.
> Clear the complete workspace first. Stopping the process releases active holding, so support the
> arms before pressing Ctrl+C. Never connect or disconnect powered hardware.

## Architecture

```mermaid
flowchart LR
    W["Windows 10.103.3.44<br/>SteamVR + umi-vive-sender"]
    S["UMI Pi 10.103.3.43<br/>trackers + cameras + buttons"]
    Y["YAM Pi 10.103.3.85<br/>safety + IK + motor control"]
    L["Left YAM<br/>USB serial to current canN"]
    R["Right YAM<br/>USB serial to current canN"]
    W -->|"broadcast tracker JSON/UDP :9001"| S
    S -->|"poses + widths + controls JSON/UDP :9140"| Y
    Y --> L
    Y --> R
```

The Windows sender broadcasts raw SteamVR poses to UDP port `9001`, so the UMI Pi and monitoring
computers on the same local network can receive the stream simultaneously. The UMI Pi converts
poses to `+X` forward, `+Y` left, `+Z` up; measures both grippers from calibrated ArUco tags; reads
the E/Z buttons; and sends one validated packet at 30 Hz. The YAM Pi owns zeroing, workspace limits,
inverse kinematics, joint limits, slew limits, CAN access, and motor commands. Images never leave
the UMI Pi.

One root `pyproject.toml` and lockfile keep the protocol and safety code on one version. Install only
the relevant hardware extra on each device: `windows`, `source`, or `yam`. Every CLI uses Tyro;
run any command with `--help` to override its typed defaults.

## Fixed Setup

| Role | Address / mapping |
|---|---|
| Windows SteamVR host | `aq@10.103.3.44`, broadcasts raw poses on UDP `9001` |
| UMI source Pi | `aq@10.103.3.43`, receives UDP `9001`, sends to `10.103.3.85:9140` |
| YAM controller Pi | `yam@10.103.3.85`, receives UDP `9140` |
| Left leader | UMI2, tracker `LHR-BBAF093A`, camera tags `2/3` |
| Right leader | UMI1, tracker `LHR-C891E882`, camera tags `0/1` |
| Left follower | CAN adapter USB serial selected at startup |
| Right follower | CAN adapter USB serial selected at startup |

The calibrated camera files and the safe I2RT gripper encoder-wrap correction are included in this
repository. No `robot-umi`, `vive-tracking`, `gello`, or patched I2RT checkout is required.

## Install

All three devices must clone the same commit from the private repository:

```text
https://github.com/AfterQuery-Research/umi-teleop.git
```

### Windows SteamVR host

In PowerShell, install [uv](https://docs.astral.sh/uv/getting-started/installation/), then:

```powershell
git clone https://github.com/AfterQuery-Research/umi-teleop.git C:\umi-teleop
cd C:\umi-teleop
uv sync --locked --extra windows --no-dev
```

Before starting the sender:

1. Plug in and power on both Vive base stations.
2. Press the center button on each Vive tracker.
3. Wait for each tracker light to turn green. Green means the tracker is paired and connected.
4. Confirm that both trackers appear in SteamVR on Windows.

### UMI source Pi

The source uses system GStreamer/PyGObject for both USB cameras and its own uv environment for the
rest:

```bash
sudo apt update
sudo apt install -y git curl v4l-utils python3-gi gir1.2-gstreamer-1.0 \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
sudo usermod -aG video,dialout aq
curl -LsSf https://astral.sh/uv/install.sh | sh
git clone https://github.com/AfterQuery-Research/umi-teleop.git /home/aq/umi-teleop
cd /home/aq/umi-teleop
uv venv --python /usr/bin/python3 --system-site-packages
uv sync --locked --extra source --no-dev
```

Reconnect to the Pi once after changing group membership.

Do not run the old recorder, Vive receiver, or button programs at the same time. They share UDP
port `9001`, GPIO `17/27`, and both cameras with `umi-source`.

### YAM controller Pi

```bash
sudo apt update
sudo apt install -y git curl build-essential cmake ninja-build can-utils
curl -LsSf https://astral.sh/uv/install.sh | sh
mkdir -p /home/yam/Documents/Repos
git clone https://github.com/AfterQuery-Research/umi-teleop.git \
  /home/yam/Documents/Repos/umi-teleop
cd /home/yam/Documents/Repos/umi-teleop
uv sync --locked --extra yam --no-dev
```

#### Discover and configure CAN

Stop robot-control processes, then list the adapters without opening a CAN bus or sending frames:

```bash
cd /home/yam/Documents/Repos/umi-teleop
.venv/bin/umi-setup-can
```

The USB serial identifies an adapter even if Linux changes its `canN` name. Determine which cable
goes to each arm and label it. Never connect or disconnect powered CAN hardware. If an adapter must
be connected by itself to establish the mapping, first shut down the Pi and remove YAM power.

Set the installation's left and right serials in the YAM shell. Replace the two example names with
the serials reported above:

```bash
export LEFT_CAN=LEFT_ADAPTER_USB_SERIAL
export RIGHT_CAN=RIGHT_ADAPTER_USB_SERIAL
```

Configure both adapters after every reboot or replug:

```bash
.venv/bin/umi-setup-can \
  --left-adapter-serial "$LEFT_CAN" \
  --right-adapter-serial "$RIGHT_CAN"

ip -details -statistics link show type can
```

This performs only `ip link` setup and verification. It does not import i2rt, open a CAN bus, send
motor frames, or move the robot. Both links must report `UP`, Classic CAN at `1000000` bit/s, and
`ERROR-ACTIVE`. You do not need to repeat setup before each run unless the Pi rebooted or an adapter
was replugged.

## Bring Up

1. Power the Pis and YAMs. Start SteamVR and wake both trackers.
2. On Windows, start tracker streaming:

   ```powershell
   cd C:\umi-teleop
   .\.venv\Scripts\umi-vive-sender.exe
   ```

   The default destination is the local broadcast address `255.255.255.255`, allowing any receiver
   bound to UDP `9001` on the same broadcast network to consume the packets. Use `--target-host`
   only when a specific unicast destination is required. The UDP stream is unauthenticated and
   unencrypted, so broadcast it only on a trusted local network.

3. In a dedicated terminal, start the UMI source and inspect its live health:

   ```bash
   ssh -t aq@10.103.3.43 \
     'cd /home/aq/umi-teleop && GPIOZERO_PIN_FACTORY=lgpio .venv/bin/umi-source'
   ```

   Keep this terminal open. Continue only when its logs show `enabled=False`, left/right
   `pose_age` and `width_age` below `0.5`, nonempty left/right `width_mm`, and `bad_vive=0`.

4. Clear the full arm workspace. In a dedicated terminal, connect to the YAM Pi, set the same two
   adapter serials used during CAN setup, and start the controller:

   ```bash
   ssh -t yam@10.103.3.85
   cd /home/yam/Documents/Repos/umi-teleop
   export LEFT_CAN=LEFT_ADAPTER_USB_SERIAL
   export RIGHT_CAN=RIGHT_ADAPTER_USB_SERIAL
   .venv/bin/umi-yam \
     --left-adapter-serial "$LEFT_CAN" \
     --right-adapter-serial "$RIGHT_CAN" \
     --confirm-clear
   ```

   It resolves each USB serial to its current `canN` name, refuses competing controllers, validates
   both CAN buses, waits for fresh disabled telemetry, and slowly moves both arms through a bent
   posture to `[0, 23, 46, 0, 0, 0]` degrees. Wait for:

   ```text
   robots holding current pose; tap the Pi ZERO button before enable
   ```

5. Hold both UMIs in the intended neutral pose, especially the right UMI, then:
   - Tap **Z** once.
   - Wait at least **5 seconds** for the post-zero enable cooldown.
   - Tap **E** once and keep the UMIs steady during the 3-second enable confirmation.
   - Begin with small motions.

The right UMI orientation at Z defines the shared command axes. Each arm otherwise follows its own
leader's position/orientation change from Z. Translation is scaled `1.5x`; the input envelope is
`0.50 m` and `90 deg`; joint speed is limited to `0.45 rad/s`. At the envelope, the controller
projects the raw UMI delta onto the boundary and stays armed. This permits up to `0.75 m` of scaled
robot translation while preventing a larger raw motion from producing an unbounded target.

## Controls

| Control | Action |
|---|---|
| E tap while disabled | Request enable; commits after 3 quiet seconds with fresh poses and widths |
| E tap while enabled | Disable immediately and hold the last commanded robot pose |
| E hold for 1 second | Cancel/disarm immediately |
| Z tap | Disable, wait 0.4 seconds for fresh poses, then capture a new zero |
| Z hold for 1 second | Ask the source and YAM controller to quit |

A new source process always creates a new session ID, so the YAM controller requires a new Z before
it can enable. Z can be used again during a session; it disarms first and captures the current UMI
and robot poses as the next references.

## Stop

1. Tap **E** once if the source is enabled; verify `enabled=False` in the source log.
2. Support both YAM arms, then press Ctrl+C in the `umi-yam` terminal. Torque holding is released.
3. Press Ctrl+C in the UMI source terminal.

4. Press Ctrl+C in the Windows sender, then power down hardware.

## Recovery

- **Startup says the source is enabled:** tap E once to disable it, verify `enabled=False`, and rerun
  `umi-yam --confirm-clear`.
- **Vive or width telemetry is stale:** make both trackers visible and both ArUco tag pairs visible;
  do not enable until both ages recover.
- **YAM controller disarms while the source still says enabled:** tap E once to synchronize the
  source back to disabled. To continue from the current robot pose, hold the UMIs neutral, tap Z,
  wait 5 seconds, then tap E. To return to the canonical posture, disable, support the arms, stop the
  controller, clear the workspace, and rerun it.
- **An exact IK target is unreachable:** each arm independently searches toward the target while
  relaxing orientation. It reports `degraded_ik` when a nearby target solves and `holding_ik` when
  it must hold that arm's last safe command. It stays armed and retries, so moving the UMI back into
  a reachable region recovers automatically. Set `--ik-hold-disarm-s` to a positive timeout only if
  a deployment should disarm after a sustained hold; the default `0` never disarms for IK alone.
- **The UMI input envelope is exceeded:** the translation vector is projected to `0.50 m` and/or
  the relative rotation is projected to `90 deg`. The controller stays armed and reports
  `input_clamped`, including raw and commanded deltas in its periodic log.
- **Packets go stale or the operator disables/cancels:** the controller disarms and holds its last
  command. It never automatically rearms after these safety events.
- **CAN is missing, down, in CAN FD mode, or at the wrong bitrate:** stop robot controllers and rerun
  `umi-setup-can` with the two adapter serials. Never use `can0`/`can1` order to infer arm side.
- **Inspect packets without motors:** run `.venv/bin/umi-yam --monitor-only` on the YAM Pi.

## Development Verification

```bash
uv sync --locked --group dev
uv run ruff format --check .
uv run ruff check .
uv run pytest
uv build
```

The test suite covers SteamVR packet construction, OpenVR coordinate conversion, strict protocol
validation, GPIO state guards, camera buffering, calibrated ArUco geometry, serial-based CAN
discovery/setup, zero/enable/session behavior, Cartesian limits, gripper mapping, the I2RT
encoder-wrap correction, and a UDP integration test across the Windows, source, and YAM layers.

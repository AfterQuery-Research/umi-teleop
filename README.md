# UMI Teleop

Standalone Vive teleoperation for two camera-equipped UMIs and two YAM robot arms.

> **Physical safety:** `umi-yam --confirm-clear` energizes both arms and holds their current joint
> positions. Automatic startup motion is opt-in through `--move-to-ready` or `--move-to-base`.
> Clear the complete workspace first. Stopping the process releases active holding, so support the
> arms before pressing Ctrl+C. Never connect or disconnect powered hardware.

## Architecture

```mermaid
flowchart LR
    W["Windows 10.103.3.44<br/>SteamVR + umi-vive-sender"]
    S["UMI Pi 10.103.3.43<br/>trackers + cameras + buttons"]
    Y["YAM Pi 10.103.3.85<br/>safety + IK + motor control"]
    C["Top RealSense + selectable side cameras<br/>optional dataset recording"]
    H["Hugging Face dataset<br/>optional private upload"]
    L["Left YAM<br/>USB serial to current canN"]
    R["Right YAM<br/>USB serial to current canN"]
    W -->|"broadcast tracker JSON/UDP :9001"| S
    S -->|"poses + widths + controls JSON/UDP :9140"| Y
    C -->|"top + left + right RGB"| Y
    Y -.->|"finalized LeRobot v3 dataset"| H
    Y --> L
    Y --> R
```

The Windows sender broadcasts raw SteamVR poses to UDP port `9001`, so the UMI Pi and monitoring
computers on the same local network can receive the stream simultaneously. The UMI Pi converts
poses to `+X` forward, `+Y` left, `+Z` up; measures both grippers from calibrated ArUco tags; reads
the E/Z buttons; and sends one validated packet at 30 Hz. The YAM Pi owns zeroing, workspace limits,
inverse kinematics, joint limits, slew limits, CAN access, and motor commands. The two UMI
ArUco-camera images never leave the UMI Pi. Optional policy-dataset images come from a top
RealSense plus either two RealSense or two ultrawide side cameras connected to the YAM Pi, and
leave it only when explicit Hub upload is enabled.

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

To record MolmoAct2-compatible LeRobot datasets on this computer, install the optional Python
3.12 recording stack as well:

```bash
uv sync --locked --extra yam --extra dataset --no-dev
```

This adds LeRobot's dataset writer and RealSense support only on the YAM computer. It is not
needed on Windows or the UMI source Pi.

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

4. Put both arms in the intended base pose and clear the full arm workspace. In a dedicated
   terminal, connect to the YAM Pi, set the same two adapter serials used during CAN setup, and
   start the controller:

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
   both CAN buses, waits for fresh telemetry, energizes the arms, and holds their current positions.
   It does not command a startup trajectory unless `--move-to-ready` or `--move-to-base` is passed
   explicitly. Wait for:

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

## Experimental LiDAR/SLAM Teleop

This path reuses the same YAM-side safety, zeroing, limits, gripper mapping, and UDP packet schema
as Vive teleop, but replaces SteamVR poses with live GLIM odometry from the RoboSense LiDAR UMIs.
It is intentionally split into small processes so the existing `umi-recorder` autostart behavior on
the UMI Pis does not change.

```mermaid
flowchart LR
    LP["LiDAR UMI Pis<br/>10.103.0.80 / 10.103.0.84"]
    LB["Lambda box<br/>ubuntu@192.222.52.98<br/>live GLIM containers"]
    MS["umi-lidar-merge-source<br/>pose :9142 + arm :9143"]
    YA["umi-yam<br/>existing YAM safety/IK"]
    LP -->|"raw LiDAR UDP via pi_tap"| LB
    LB -->|"LiDAR body poses JSON/UDP :9142"| MS
    LP -->|"gripper widths + buttons JSON/UDP :9143"| MS
    MS -->|"existing teleop JSON/UDP :9140"| YA
```

Known current LiDAR hardware from `umi-flow`:

| Role | Address / notes |
|---|---|
| Lambda processing box | `ubuntu@192.222.52.98` |
| Right LiDAR UMI (`umi1`) | `umi1@10.103.0.80`, `umi-recorder`, `umi-ptp@eth0`, and upload tunnel autostart |
| Left LiDAR UMI (`umi2`) | `umi2@10.103.0.84`, `umi-recorder`, `umi-ptp@eth0`, and upload tunnel autostart |
| LiDAR network | each Pi's `eth0` uses host `192.168.1.102`, LiDAR `192.168.1.200` |

Current side assignment: `umi1` is the right-hand leader and `umi2` is the left-hand leader. The
scripts in `scripts/` bake this in, while the lower-level CLIs still allow explicit overrides.

Observed live Pi configs at the time this was written:

| Pi | Side | Config name | Camera calibration | ArUco tags |
|---|---|---|---|---|
| `10.103.0.80` | right | `umi1` | `config/calib/umi1_calib.npz` | `2 3` |
| `10.103.0.84` | left | `umi2` | `config/calib/umi2_calib.npz` | `0 1` |

### What must be running first

1. The Lambda box must have one live GLIM container per UMI, each named distinctly.
   `scripts/run-lambda-live-glim-containers.sh` builds the live image and starts containers named
   `glim-10-103-0-80` and `glim-10-103-0-84`.
2. The LiDARs must be powered on before starting the GLIM feed. If the LiDARs are off, the merge
   source will stay disabled because pose ages never become fresh.
3. The YAM side should be run with the LiDAR TCP lever instead of the Vive tracker lever:

   ```bash
   scripts/run-lidar-yam.sh
   ```

   The script binds `umi-yam` to `127.0.0.1:9140` by default so only the local LiDAR merger can feed
   the robot controller. This prevents stale LAN senders, for example an old Vive UMI Pi process,
   from interleaving disabled packets with the merged LiDAR stream. Override with
   `YAM_TELEOP_BIND=0.0.0.0` only if you intentionally run the merger on another machine. The script
   defaults to the observed CAN mapping: left `209D378045465008`, right `206237A845465006`. Override
   with `LEFT_CAN=... RIGHT_CAN=...` if the hardware mapping changes.

### Bring-up shape

Pick a machine for the merger. The YAM Pi is preferred because the merger can send to
`127.0.0.1:9140`, keeping the robot-control port closed to stale LAN senders. An operator laptop
also works if `umi-yam` is intentionally started with `YAM_TELEOP_BIND=0.0.0.0` and the merger sends
to the YAM Pi's LAN IP.

1. Start the merger:

   ```bash
   scripts/run-lidar-merge-yam.sh
   ```

2. On Lambda, build and start the live GLIM containers and Pi UDP feeds:

   ```bash
   cd /home/ubuntu/umi-teleop
   scripts/run-lambda-live-glim-containers.sh
   ```

   To prepare the Docker image without touching live feeds:

   ```bash
   BUILD_ONLY=1 scripts/run-lambda-live-glim-containers.sh
   ```

3. Stream live GLIM poses to the merger.

   In the observed AfterQuery lab network, Lambda can be reached by SSH from the operator laptop,
   but cannot send UDP directly to the private `10.103.*` YAM LAN. In that topology, run this relay
   on the operator laptop/Mac:

   ```bash
   cd /path/to/umi-teleop
   scripts/run-lidar-lambda-pose-ssh-relay.sh ubuntu@192.222.52.98 10.103.3.85 9142
   ```

   The relay SSHes into Lambda, starts the live GLIM pose sender in stdout-packet mode, and forwards
   those datagrams from the laptop to the YAM Pi's merger UDP port. If Lambda and the merger are on a
   network where UDP routing works directly, you can instead run this on Lambda:

   ```bash
   cd /home/ubuntu/umi-teleop
   scripts/run-lidar-lambda-pose-sender.sh MERGER_HOST
   ```

   `umi-lidar-lambda` sends the LiDAR body pose in the UMI-handle frame. It does **not** pre-apply
   the TCP lever unless `--include-tcp-offset` is set for debugging.

4. On each UMI Pi, enter teleop mode for this session and send gripper/button telemetry. The wrapper
   refuses to stop `umi-recorder.service` unless explicitly confirmed, and it does not change whether
   the recorder autostarts on the next boot.

   On the right/`umi1` Pi:

   ```bash
   cd /home/aq/umi-teleop
   scripts/run-lidar-teleop-umi1-right.sh MERGER_HOST
   ```

   On the left/`umi2` Pi:

   ```bash
   cd /home/aq/umi-teleop
   scripts/run-lidar-teleop-umi2-left.sh MERGER_HOST
   ```

5. Once the merger logs show fresh left/right `pose_age`, fresh left/right `width_age`, and
   nonempty widths, start `umi-yam`:

   ```bash
   cd /home/yam/Documents/Repos/umi-teleop
   scripts/run-lidar-yam.sh
   ```

   Then use the same controls as Vive teleop: Z tap to zero, wait for cooldown, E tap to enable.

### Returning the UMI Pis to recorder mode

Stopping `umi-lidar-teleop-mode` leaves boot/autostart policy unchanged. If you want the recorder
back immediately without rebooting, either pass `--restart-recorder-on-exit` when launching teleop
mode, or run:

```bash
sudo systemctl start umi-recorder.service
```

## Controls

| Control | Action |
|---|---|
| E tap while disabled | Request enable; commits after 3 quiet seconds with fresh poses and widths |
| E tap while enabled | Disable immediately; a recording run saves, then returns both arms to base |
| E hold for 1 second | Cancel/disarm immediately |
| Z tap | Disable, wait 0.4 seconds for fresh poses, then capture a new zero |
| Z hold for 1 second | Ask the source and YAM controller to quit |

A new source process always creates a new session ID, so the YAM controller requires a new Z before
it can enable. Z can be used again during a session; it disarms first and captures the current UMI
and robot poses as the next references.

## Current LiDAR teleop envelope

The default `scripts/run-lidar-yam.sh` LiDAR/YAM controller parameters are:

| Parameter | Value |
|---|---:|
| Position scale | `1.5x` |
| Input translation safety limit | `0.50 m` before scaling |
| Effective commanded translation cap | up to `0.75 m`, if IK and joint limits allow |
| Rotation safety limit | `90 deg` |
| Joint speed cap | `0.45 rad/s` |
| Gripper speed cap | `1.50` normalized open/close units per second |

The controller clamps larger input to this envelope and stays armed. Unreachable Cartesian targets
use the same degraded/holding IK policy as Vive teleop. For initial bring-up, keep each UMI motion
small (about `2-3 cm` translation and under `10-15 deg` wrist rotation), then expand gradually.

## Record A LeRobot Dataset

Recording is optional and runs inside `umi-yam`, which remains the only process allowed to own the
YAM CAN buses. Do not run `lerobot-record`, policy inference, or another `umi-yam` at the same time.
The UMI source Pi keeps owning its two ArUco cameras; the YAM computer separately owns three
RealSense policy cameras.

First discover and identify the RealSense cameras from the LeRobot checkout:

```bash
cd /home/yam/Documents/Repos/lerobot
uv run lerobot-find-cameras realsense
```

Inspect the saved previews, then assign three distinct serials in the fixed MolmoAct2 view order:

```bash
export TOP_CAMERA=TOP_REALSENSE_SERIAL
export LEFT_CAMERA=LEFT_REALSENSE_SERIAL
export RIGHT_CAMERA=RIGHT_REALSENSE_SERIAL
```

The top D435 is captured at RGB `640x480` at 30 Hz, center-cropped to `640x360`, and downsampled to
`480x270`. The left and right D405 cameras are captured natively at RGB `480x270` at 30 Hz. This
keeps the same 16:9 views while reducing encoder load; the MolmoAct2 checkpoint resizes each view to
its `378x378` image input. Recording validates and opens all three before it opens CAN or constructs
either robot, and requires at least 5 GiB of free dataset storage before each episode. Authenticate
once before requesting upload; the token stays in Hugging Face's normal credential store and is
never passed to `umi-yam`:

```bash
cd /home/yam/Documents/Repos/umi-teleop
.venv/bin/hf auth login
```

Choose a new Hub repository ID and a local path that does not exist yet, clear the robot workspace,
then run:

```bash
.venv/bin/umi-yam \
  --left-adapter-serial "$LEFT_CAN" \
  --right-adapter-serial "$RIGHT_CAN" \
  --dataset.repo-id AfterQuery-Research/YOUR_DATASET_NAME \
  --dataset.root /home/yam/Datasets/YOUR_DATASET_NAME \
  --dataset.task "Pick up the oranges and put them in the bowl" \
  --dataset.top-camera-serial "$TOP_CAMERA" \
  --dataset.left-camera-serial "$LEFT_CAMERA" \
  --dataset.right-camera-serial "$RIGHT_CAMERA" \
  --dataset.streaming-encoding \
  --dataset.push-to-hub \
  --confirm-clear
```

To retain the top D435 while replacing the two D405 wrist views with the ultrawide cameras, select
the additive ultrawide side-camera mode. The existing RealSense mode remains the default. The
ultrawides are captured as MJPEG at RGB `640x480`, then downsampled and stored at RGB `416x312`;
the cameras require a requested 60 fps setting to deliver the measured 30 fps stream. They have
identical USB serials, so use their stable physical USB paths:

```bash
export LEFT_WIDE=/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.4.4.4:1.0-video-index0
export RIGHT_WIDE=/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1.4.4.3:1.0-video-index0

.venv/bin/umi-yam \
  --left-adapter-serial "$LEFT_CAN" \
  --right-adapter-serial "$RIGHT_CAN" \
  --dataset.repo-id AfterQuery-Research/YOUR_ULTRAWIDE_DATASET_NAME \
  --dataset.root /home/yam/Datasets/YOUR_ULTRAWIDE_DATASET_NAME \
  --dataset.task "Pick up the oranges and put them in the bowl" \
  --dataset.top-camera-serial "$TOP_CAMERA" \
  --dataset.side-camera-kind ultrawide \
  --dataset.left-camera-path "$LEFT_WIDE" \
  --dataset.right-camera-path "$RIGHT_WIDE" \
  --dataset.streaming-encoding \
  --dataset.push-to-hub \
  --confirm-clear
```

Use a new dataset root and Hub repository when switching side-camera kinds. The RealSense schema
stores all views at `480x270`; the ultrawide schema stores `top` at `480x270` and `left`/`right` at
`416x312`, so the two modes cannot be resumed into the same LeRobot dataset.

The default launch does not command or record a startup trajectory. Recording begins on the first
successful E enable. Tapping E to stop saves the episode, then automatically returns both arms to
the all-zero base posture without recording the return. The previous Cartesian ZERO is invalidated;
after the terminal prints `robots at unrecorded base pose`, place the UMIs at their next neutral
poses, tap Z, wait for the cooldown, and tap E to record another episode without restarting any
process. The idle wait for ZERO/E is not recorded. An automatic, recorded teleop-ready startup
remains available only when `--move-to-ready` is passed explicitly; an unrecorded startup parking
motion remains available only with `--move-to-base`. Z while armed, CANCEL, a safety disarm, camera
failure, sustained timing failure, source restart, or Ctrl+C during an episode discards that
incomplete episode and holds the last safe pose rather than initiating an automatic return. A
transient scheduling backlog gets 0.5 seconds to recover before it can invalidate the episode.
Ordinary degraded or holding IK records the final bounded/held command without ending the episode.
After the desired episode has printed `DATASET episode saved` and the arms have returned to base,
support the arms and press Ctrl+C. Both robots close first; the dataset then finalizes, validates
locally, and uploads privately. The local dataset is retained if login or upload fails.

To append during a later run, use the same repo ID and root and add `--dataset.resume`. Omit
`--dataset.push-to-hub` to collect and validate locally without uploading. H.264/yuv420p is the
Raspberry Pi default. Streaming H.264 uses a 30-frame GOP and the `ultrafast` x264 preset so the
three camera encoders can keep up with capture without starving arm control. RGB statistics are
computed from the completed videos after an episode ends instead of competing with live encoding,
so saving an episode can take a little longer. If the
background writer reaches 90% of its queue, the current episode is discarded and teleoperation is
disarmed while the controller keeps the motors holding position. Add
`--dataset.video-codec libsvtav1` only when AV1 file-level parity is required and the slower encoder
has been tested on this hardware. Final validation decodes the
first and last saved frames with PyAV, avoiding TorchCodec's optional GPU-linked runtime on ARM64.
It also requires every camera video file to contain exactly the combined number of frames declared
by the episodes that reference it; LeRobot may pack multiple episodes into one shared file. A
streaming encoder drop discards the active episode instead of saving misaligned views.
Resume only a dataset created with the same selected camera profile. Recording rejects an existing
root whose per-camera shapes or frame rate do not match.

The resulting LeRobot v3 schema keeps the `MolmoAct2-BimanualYAM-Dataset` camera, state, and action
semantics: `top`, `left`, and `right` RGB images plus 14-value `observation.state` and `action`
vectors ordered as left joints 0-5, left gripper, right joints 0-5, right gripper. State is measured
immediately before control. Action is the final bounded target successfully sent to both arms.

If upload fails after the robots have closed, retry it without opening cameras or CAN:

```bash
.venv/bin/umi-dataset-upload \
  --repo-id AfterQuery-Research/YOUR_DATASET_NAME \
  --root /home/yam/Datasets/YOUR_DATASET_NAME
```

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
  source back to disabled. To continue from the current robot pose after a safety disarm, hold the
  UMIs neutral, tap Z, wait 5 seconds, then tap E. An intentional recording stop returns to the
  all-zero base posture automatically and then requires a new Z.
- **An exact IK target is unreachable:** each arm independently searches toward the target while
  relaxing orientation. It reports `degraded_ik` when a nearby target solves and `holding_ik` when
  it must hold that arm's last safe command. IK runs in one latest-target worker per arm, so a slow
  fallback search cannot stall 30 Hz observation, command, or dataset capture. The foreground loop
  rejects an IK result older than `--ik-result-max-age-s` (default `0.25`) and continues holding the
  last validated goal. It stays armed and retries, so moving the UMI back into a reachable region
  recovers automatically. Set `--ik-hold-disarm-s` to a positive timeout only if a deployment should
  disarm after a sustained hold; the default `0` never disarms for IK alone.
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
encoder-wrap correction, dataset episode and schema behavior, and UDP integration across the
Windows, source, YAM, and recorder layers. With the `dataset` extra installed, it also writes,
finalizes, reopens, and decodes a real local LeRobot v3 video dataset.

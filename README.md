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
    C["3 RealSense policy cameras<br/>optional dataset recording"]
    H["Hugging Face dataset<br/>optional private upload"]
    L["Left YAM<br/>USB serial to current canN"]
    R["Right YAM<br/>USB serial to current canN"]
    W -->|"raw tracker JSON/UDP :9001"| S
    S -->|"poses + widths + controls JSON/UDP :9140"| Y
    C -->|"top + left + right RGB"| Y
    Y -.->|"finalized LeRobot v3 dataset"| H
    Y --> L
    Y --> R
```

The Windows sender publishes raw SteamVR poses. The UMI Pi converts them to `+X` forward, `+Y`
left, `+Z` up; measures both grippers from calibrated ArUco tags; reads the E/Z buttons; and sends
one validated packet at 30 Hz. The YAM Pi owns zeroing, workspace limits, inverse kinematics, joint
limits, slew limits, CAN access, and motor commands. The two UMI ArUco-camera images never leave
the UMI Pi. Optional policy-dataset images come from three separate RealSense cameras connected to
the YAM Pi and leave it only when explicit Hub upload is enabled.

One root `pyproject.toml` and lockfile keep the protocol and safety code on one version. Install only
the relevant hardware extra on each device: `windows`, `source`, or `yam`. Every CLI uses Tyro;
run any command with `--help` to override its typed defaults.

## Fixed Setup

| Role | Address / mapping |
|---|---|
| Windows SteamVR host | `aq@10.103.3.44` |
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
`0.40 m` and `90 deg`; joint speed is limited to `0.45 rad/s`.

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

All three cameras must support RGB `640x360` at 30 Hz. Recording validates and opens them before it
opens CAN or constructs either robot, and requires at least 5 GiB of free dataset storage before
each episode. Authenticate once before requesting upload; the token stays in Hugging Face's normal
credential store and is never passed to `umi-yam`:

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
  --dataset.push-to-hub \
  --confirm-clear
```

Use the normal teleoperation controls. A successful E enable starts an episode. Tapping E to stop
saves it. Z, CANCEL, a safety disarm, camera/IK/timing failure, source restart, or Ctrl+C during an
episode discards that incomplete episode. After the desired episode has printed `DATASET episode
saved`, support the arms and press Ctrl+C. Both robots close first; the dataset then finalizes,
validates locally, and uploads privately. The local dataset is retained if login or upload fails.

To append during a later run, use the same repo ID and root and add `--dataset.resume`. Omit
`--dataset.push-to-hub` to collect and validate locally without uploading. H.264/yuv420p is the
Raspberry Pi default; add `--dataset.video-codec libsvtav1` only when AV1 file-level parity is
required and the slower encoder has been tested on this hardware.

The resulting LeRobot v3 schema matches `MolmoAct2-BimanualYAM-Dataset`: `top`, `left`, and `right`
RGB images plus 14-value `observation.state` and `action` vectors ordered as left joints 0-5, left
gripper, right joints 0-5, right gripper. State is measured immediately before control. Action is
the final bounded target successfully sent to both arms.

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
  source back to disabled. To continue from the current robot pose, hold the UMIs neutral, tap Z,
  wait 5 seconds, then tap E. To return to the canonical posture, disable, support the arms, stop the
  controller, clear the workspace, and rerun it.
- **IK fails five consecutive cycles, a workspace limit is exceeded, packets go stale, or a joint
  target approaches its limit:** the controller disarms and holds its last command. It never
  automatically rearms.
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

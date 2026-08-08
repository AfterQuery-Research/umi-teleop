# umi-teleop-fixed

A standalone UMI teleoperation stack for the YAM robot on `aq-yam`. It uses
i2rt directly and does not import or depend on LeRobot.

## Setup

```bash
cd ~/Documents/Repos/umi-teleop-fixed
export PATH="$HOME/.local/bin:$PATH"
uv sync --dev
source .env
```

The ignored `.env` on `aq-yam` exports `ROBOT_ID`, `LEFT_CAN`, and `RIGHT_CAN`.
CAN adapters are always selected by USB serial because Linux `canN` names can
change after a reboot or USB replug.

## CAN

Discover adapters without changing link state or sending CAN frames:

```bash
uv run umi-setup-can
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

## Development

```bash
uv run ruff check .
uv run pytest
```

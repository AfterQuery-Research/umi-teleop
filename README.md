# umi-teleop-fixed

A standalone UMI teleoperation stack for the YAM robot on `aq-yam`. It uses
i2rt directly and does not import or depend on LeRobot.

## CAN

CAN adapters are selected by USB serial because Linux `canN` names can change
after a reboot or USB replug. Discover the connected serials without changing
link state or sending CAN frames:

```bash
uv run umi-setup-can
```

The current `aq-yam` mapping is:

| Side | Adapter serial |
| --- | --- |
| Left | `209D378045465008` |
| Right | `206237A845465006` |

Configure and verify both classic CAN links at 1 Mbps:

```bash
uv run umi-setup-can \
    --left-adapter-serial 209D378045465008 \
    --right-adapter-serial 206237A845465006
```

Run this after a reboot or adapter replug. Do not start robot control unless
both links report `UP/LOWER_UP`, `ERROR-ACTIVE`, `1000000`, and `Classic CAN`.

## Gripper calibration

Calibration moves one gripper through its full travel. Clear the workspace and
keep hands away.

```bash
uv run umi-calibrate-gripper \
    --robot-id afterquery-dual-yam-01 \
    --adapter-serial 209D378045465008 \
    --side left

uv run umi-calibrate-gripper \
    --robot-id afterquery-dual-yam-01 \
    --adapter-serial 206237A845465006 \
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

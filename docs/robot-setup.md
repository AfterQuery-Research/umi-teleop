# YAM robot setup on aq-yam

The YAM arms use classic CAN at 1 Mbps. Linux interface names can swap after a
reboot or USB replug, so always select adapters by USB serial.

## 1. Install the environment

```bash
cd ~/Documents/Repos/umi-teleop-fixed
export PATH="$HOME/.local/bin:$PATH"
uv sync --dev
```

## 2. Discover the adapters

```bash
uv run umi-setup-can
```

With no arguments, this is read-only: it lists SocketCAN interfaces and sends no
CAN frames.

The current `aq-yam` mapping is:

| Arm | USB adapter serial |
| --- | --- |
| Left | `209D378045465008` |
| Right | `206237A845465006` |

## 3. Configure both links

Run this after a reboot or whenever either USB-to-CAN adapter is replugged:

```bash
uv run umi-setup-can \
    --left-adapter-serial 209D378045465008 \
    --right-adapter-serial 206237A845465006
```

The command resolves the adapters' current `canN` names, configures classic CAN
at 1 Mbps, brings both links up, and verifies their state. It uses only `ip link`;
it does not import robot-control code, send CAN frames, or move the robot.
Links that already pass every readiness check are verified without being bounced.

## 4. Verify before robot control

```bash
ip -details link show type can
```

Both interfaces must report all of the following:

- `UP` and `LOWER_UP`
- `state UP`
- CAN state `ERROR-ACTIVE`
- `bitrate 1000000`
- classic CAN, not CAN FD

Do not start robot control if an interface is missing or reports `DOWN`,
`STOPPED`, `ERROR-PASSIVE`, `BUS-OFF`, or a different bitrate.

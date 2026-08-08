# umi-teleop-fixed

A clean UMI teleoperation implementation for the YAM robot on `aq-yam`.

The first supported operations are safe SocketCAN setup and supervised YAM
gripper calibration. Hardware access uses i2rt directly; this package does not
import or depend on LeRobot.

## Development

```bash
export PATH="$HOME/.local/bin:$PATH"
uv sync --dev
uv run ruff check .
uv run pytest
```

## Robot setup

Follow [docs/robot-setup.md](docs/robot-setup.md).

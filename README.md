# umi-teleop-fixed

A clean UMI teleoperation implementation for the YAM robot on `aq-yam`.

The first supported operation is safe SocketCAN discovery and setup. It resolves
the arms by USB adapter serial rather than assuming that `can0` is always left.

## Development

```bash
export PATH="$HOME/.local/bin:$PATH"
uv sync --dev
uv run ruff check .
uv run pytest
```

## Robot setup

Follow [docs/robot-setup.md](docs/robot-setup.md).


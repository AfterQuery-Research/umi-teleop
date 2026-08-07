"""One-command native Linux teleoperation launcher for the dual YAM test rig."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import tyro

DEFAULT_LEFT_CAN = "209D378045465008"
DEFAULT_RIGHT_CAN = "206237A845465006"


@dataclass(frozen=True)
class Config:
    """Start native Vive tracking and arm-only dual-YAM control on this machine."""

    left_adapter_serial: str = os.environ.get("LEFT_CAN", DEFAULT_LEFT_CAN)
    right_adapter_serial: str = os.environ.get("RIGHT_CAN", DEFAULT_RIGHT_CAN)
    left_tracker_serial: str = "LHR-BBAF093A"
    right_tracker_serial: str = "LHR-C891E882"
    confirm_clear: bool = False
    """Confirm that the complete physical arm workspace is clear."""
    simulate: bool = False
    """Use I2RT simulation and never open CAN."""


def build_commands(config: Config, bin_dir: Path | None = None) -> dict[str, list[str]]:
    if sys.platform != "linux":
        raise ValueError("native local teleoperation requires Linux")
    if not config.simulate and not config.confirm_clear:
        raise ValueError("clear the complete arm workspace and pass --confirm-clear")
    if not config.left_tracker_serial.strip() or not config.right_tracker_serial.strip():
        raise ValueError("left and right tracker serials must not be empty")
    if config.left_tracker_serial.casefold() == config.right_tracker_serial.casefold():
        raise ValueError("left and right tracker serials must be different")
    if not config.simulate:
        if not config.left_adapter_serial.strip() or not config.right_adapter_serial.strip():
            raise ValueError("left and right CAN adapter serials must not be empty")
        if config.left_adapter_serial.casefold() == config.right_adapter_serial.casefold():
            raise ValueError("left and right CAN adapter serials must be different")

    binaries = Path(sys.executable).resolve().parent if bin_dir is None else bin_dir
    commands = {
        "tracking": [str(binaries / "umi-vive-linux"), "--rate-hz", "60"],
        "source": [
            str(binaries / "umi-source"),
            "--arm-only",
            "--buttons",
            "keyboard",
            "--left-serial",
            config.left_tracker_serial,
            "--right-serial",
            config.right_tracker_serial,
            "--send-hz",
            "60",
            "--max-pose-age-s",
            "0.20",
            "--enable-confirm-s",
            "1.0",
            "--enable-cooldown-s",
            "0.5",
        ],
        "controller": [
            str(binaries / "umi-yam"),
            "--arm-only",
            "--hz",
            "50",
            "--max-packet-age",
            "0.25",
            "--max-pose-age",
            "0.20",
        ],
    }
    if config.simulate:
        commands["controller"].append("--simulate")
    else:
        commands["controller"].extend(
            [
                "--left-adapter-serial",
                config.left_adapter_serial,
                "--right-adapter-serial",
                config.right_adapter_serial,
                "--confirm-clear",
            ]
        )
    return commands


def stop_processes(processes: list[tuple[str, Any]], timeout_s: float = 5.0) -> None:
    for _name, process in reversed(processes):
        if process.poll() is None:
            process.send_signal(signal.SIGINT)

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and any(
        process.poll() is None for _name, process in processes
    ):
        time.sleep(0.05)

    for _name, process in reversed(processes):
        if process.poll() is None:
            process.terminate()
    for _name, process in reversed(processes):
        if process.poll() is None:
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)


def run(
    config: Config,
    *,
    popen: Callable[..., Any] = subprocess.Popen,
) -> int:
    commands = build_commands(config)
    for name, command in commands.items():
        executable = Path(command[0])
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise RuntimeError(f"missing {name} executable: {executable}")

    print("UMI TELEOP TEST: no dataset recording and no gripper input", flush=True)
    print(
        "Controls: Z + Enter = zero, E + Enter = start, S + Enter = stop, Q + Enter = quit",
        flush=True,
    )
    processes: list[tuple[str, Any]] = []
    try:
        for name in ("tracking", "source", "controller"):
            print(f"starting {name}...", flush=True)
            processes.append((name, popen(commands[name])))

        while True:
            for name, process in processes:
                return_code = process.poll()
                if return_code is not None:
                    if return_code != 0:
                        print(f"{name} exited with status {return_code}", file=sys.stderr)
                    return int(return_code)
            time.sleep(0.1)
    except KeyboardInterrupt:
        return 130
    finally:
        stop_processes(processes)


def main() -> None:
    try:
        raise SystemExit(run(tyro.cli(Config)))
    except (RuntimeError, ValueError) as exc:
        print(f"TELEOP START ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()

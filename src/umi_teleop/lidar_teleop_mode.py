"""Guarded UMI-Pi entrypoint for LiDAR teleop mode.

This does not enable/disable any boot services.  It only stops the currently running recorder
service for the duration of this process, because the recorder owns the camera/buttons that
teleop needs.
"""

from __future__ import annotations

import subprocess
from dataclasses import dataclass
from typing import Literal

import tyro

from umi_teleop.lidar_arm_source import Config as ArmSourceConfig
from umi_teleop.lidar_arm_source import run as run_arm_source


@dataclass
class Config:
    target_host: str
    arm: Literal["left", "right"]
    source_id: str = ""
    target_port: int = 9143
    camera_device: str = "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0"
    calib: str = "config/calib/umi1_calib.npz"
    tag_ids: tuple[int, int] = (0, 1)
    width: int = 800
    height: int = 600
    camera_fps: int = 30
    exposure: int = 100
    detect_hz: float = 15.0
    send_hz: float = 30.0
    roi_top_frac: float = 0.0
    buttons: Literal["none", "gpio", "keyboard"] = "none"
    record_pin: int = 12
    quit_pin: int = 16
    button_hold_s: float = 1.0
    recorder_service: str = "umi-recorder.service"
    confirm_stop_recorder: bool = False
    restart_recorder_on_exit: bool = False


def _systemctl(action: str, service: str, *, check: bool = True) -> subprocess.CompletedProcess:
    cmd = ["sudo", "systemctl", action, service]
    return subprocess.run(cmd, check=check, text=True, capture_output=True)


def _is_active(service: str) -> bool:
    proc = _systemctl("is-active", service, check=False)
    return proc.stdout.strip() == "active"


def run(cfg: Config) -> None:
    was_active = _is_active(cfg.recorder_service)
    if was_active:
        if not cfg.confirm_stop_recorder:
            raise SystemExit(
                f"{cfg.recorder_service} is active and owns the camera/buttons. "
                f"Rerun with --confirm-stop-recorder to enter teleop mode for this session."
            )
        print(f"Stopping {cfg.recorder_service} for teleop mode...", flush=True)
        _systemctl("stop", cfg.recorder_service)
    try:
        run_arm_source(
            ArmSourceConfig(
                target_host=cfg.target_host,
                arm=cfg.arm,
                source_id=cfg.source_id,
                target_port=cfg.target_port,
                camera_device=cfg.camera_device,
                calib=cfg.calib,
                tag_ids=cfg.tag_ids,
                width=cfg.width,
                height=cfg.height,
                camera_fps=cfg.camera_fps,
                exposure=cfg.exposure,
                detect_hz=cfg.detect_hz,
                send_hz=cfg.send_hz,
                roi_top_frac=cfg.roi_top_frac,
                buttons=cfg.buttons,
                record_pin=cfg.record_pin,
                quit_pin=cfg.quit_pin,
                button_hold_s=cfg.button_hold_s,
            )
        )
    finally:
        if was_active and cfg.restart_recorder_on_exit:
            print(f"Restarting {cfg.recorder_service}...", flush=True)
            _systemctl("start", cfg.recorder_service)


def main() -> None:
    run(tyro.cli(Config))


if __name__ == "__main__":
    main()

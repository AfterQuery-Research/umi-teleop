"""Per-UMI Pi gripper/button source for LiDAR teleoperation."""

from __future__ import annotations

import logging
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import tyro

from umi_teleop.buttons import GpioButtons, KeyboardButtons
from umi_teleop.camera import LocalGstCamera
from umi_teleop.gripper import GripperConfig, GripperGeometry, detector_from_config
from umi_teleop.lidar import encode_lidar_arm_packet
from umi_teleop.protocol import ButtonState, WidthWorker


@dataclass
class Config:
    target_host: str
    """Host running umi-lidar-merge-source."""
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


class NoButtons:
    def poll(self):
        return None


def _button_source(cfg: Config):
    if cfg.buttons == "none":
        return NoButtons()
    if cfg.buttons == "keyboard":
        return KeyboardButtons()
    return GpioButtons(
        record_pin=cfg.record_pin,
        quit_pin=cfg.quit_pin,
        hold_s=cfg.button_hold_s,
    )


def run(cfg: Config) -> None:
    if cfg.send_hz <= 0 or cfg.detect_hz <= 0:
        raise ValueError("send_hz and detect_hz must be positive")
    calib = Path(cfg.calib)
    if not calib.is_file():
        raise FileNotFoundError(f"calibration file not found: {calib}")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    log = logging.getLogger("umi-lidar-arm-source")
    source_id = cfg.source_id or cfg.arm

    camera = LocalGstCamera(
        cfg.camera_device,
        cfg.width,
        cfg.height,
        cfg.camera_fps,
        cfg.exposure,
    )
    detector = detector_from_config(
        GripperConfig(
            geom=GripperGeometry(),
            roi_top_frac=cfg.roi_top_frac,
            calib_path=calib,
        ),
        cfg.width,
        cfg.height,
        target_ids=cfg.tag_ids,
    )
    worker = WidthWorker(camera, detector, detect_hz=cfg.detect_hz)
    buttons = _button_source(cfg)
    state = ButtonState()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (cfg.target_host, cfg.target_port)
    period = 1.0 / cfg.send_hz
    seq = 0
    last_status = 0.0
    log.info(
        "arm=%s source_id=%s -> %s:%d camera=%s tags=%s buttons=%s session=%s",
        cfg.arm,
        source_id,
        cfg.target_host,
        cfg.target_port,
        cfg.camera_device,
        cfg.tag_ids,
        cfg.buttons,
        state.session_id,
    )
    try:
        worker.start()
        while True:
            started = time.monotonic()
            while (event := buttons.poll()) is not None:
                state.handle(event)
                log.info(
                    "button=%s requested_enabled=%s zero=%d toggle=%d cancel=%d quit=%d",
                    event.name,
                    state.requested_enabled,
                    state.zero_id,
                    state.toggle_id,
                    state.cancel_id,
                    state.quit_id,
                )
            width = worker.snapshot()
            packet = {
                "version": 1,
                "session_id": state.session_id,
                "source_id": source_id,
                "arm": cfg.arm,
                "seq": seq,
                "requested_enabled": state.requested_enabled,
                "zero_id": state.zero_id,
                "toggle_id": state.toggle_id,
                "cancel_id": state.cancel_id,
                "quit_id": state.quit_id,
                "width_mm": width["width_mm"],
                "width_age_s": width["age_s"],
            }
            sock.sendto(encode_lidar_arm_packet(packet), target)
            seq += 1
            now = time.monotonic()
            if now - last_status >= 1.0:
                log.info(
                    "seq=%d width_mm=%s width_age=%s frames=%d detections=%d misses=%d errors=%d",
                    seq,
                    "--" if width["width_mm"] is None else f"{width['width_mm']:.1f}",
                    "--" if width["age_s"] is None else f"{width['age_s']:.3f}",
                    worker.frames_seen,
                    worker.detections,
                    worker.misses,
                    worker.errors,
                )
                last_status = now
            if state.quitting:
                break
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    finally:
        worker.stop()
        close = getattr(buttons, "close", None)
        if close is not None:
            close()


def main() -> None:
    run(tyro.cli(Config))


if __name__ == "__main__":
    main()

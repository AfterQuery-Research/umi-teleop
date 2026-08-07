"""Aggregate Vive poses, optional ArUco widths, and controls for the YAM controller.

In the all-local Linux setup it receives native Vive UDP on loopback, reads keyboard controls, and
emits validated telemetry to ``umi-yam`` on the same host. The legacy Pi setup may also enable its
two USB cameras and GPIO buttons. This process never opens CAN or commands a robot.
"""

from __future__ import annotations

import logging
import socket
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import tyro

from umi_teleop.buttons import GpioButtons, KeyboardButtons
from umi_teleop.protocol import (
    ARM_NAMES,
    ButtonState,
    EnableRequestGuard,
    VivePacketReceiver,
    WidthWorker,
    ZeroRequestGuard,
    encode_teleop_packet,
)
from umi_teleop.types import Event

UMI1_DEVICE = "/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0"
UMI2_DEVICE = "/dev/v4l/by-path/platform-xhci-hcd.1-usb-0:2:1.0-video-index0"
DATA_DIR = Path(__file__).resolve().parent / "data"


@dataclass
class Config:
    target_host: str = "127.0.0.1"
    """YAM controller address; loopback is the all-local Linux default."""
    target_port: int = 9140
    """YAM controller UDP port."""
    vive_bind: str = "0.0.0.0"
    vive_port: int = 9001
    """Windows sender destination port."""
    # Physical convention: UMI1 is the operator's right-side leader; UMI2 is left.
    left_serial: str = "LHR-BBAF093A"
    right_serial: str = "LHR-C891E882"
    left_device: str = UMI2_DEVICE
    right_device: str = UMI1_DEVICE
    width: int = 800
    height: int = 600
    camera_fps: int = 30
    exposure: int = 100
    detect_hz: float = 15.0
    send_hz: float = 30.0
    left_calib: str = str(DATA_DIR / "umi2_calib.npz")
    right_calib: str = str(DATA_DIR / "umi1_calib.npz")
    left_tag_ids: tuple[int, int] = (2, 3)
    right_tag_ids: tuple[int, int] = (0, 1)
    roi_top_frac: float = 0.0
    max_pose_age_s: float = 0.25
    max_width_age_s: float = 0.5
    enable_confirm_s: float = 3.0
    enable_cooldown_s: float = 5.0
    button_hold_s: float = 1.0
    zero_settle_s: float = 0.4
    zero_timeout_s: float = 4.0
    buttons: Literal["gpio", "keyboard"] = "gpio"
    record_pin: int = 17
    quit_pin: int = 27
    quit_burst_s: float = 0.75
    arm_only: bool = False
    """Skip gripper cameras and require only fresh Vive poses for ZERO and enable."""
    quiet: bool = False
    """Show only controls, state changes, warnings, and errors."""


def _calib(path: str) -> Path:
    resolved = Path(path)
    if not resolved.is_file():
        raise FileNotFoundError(f"required gripper calibration not found: {resolved}")
    return resolved


def _status_value(value: dict, key: str) -> str:
    item = value.get(key)
    return "--" if item is None else f"{float(item):.3f}"


def _tracking_status(poses: dict[str, dict[str, Any]], serials: dict[str, str]) -> str:
    parts = []
    for name in ARM_NAMES:
        pose = poses.get(name)
        age = "missing" if pose is None else f"age={float(pose['age_s']):.3f}s"
        parts.append(f"{name}({serials[name]})={age}")
    return " ".join(parts)


def run(cfg: Config) -> None:
    if (
        cfg.send_hz <= 0
        or cfg.max_pose_age_s <= 0
        or cfg.max_width_age_s <= 0
        or cfg.enable_confirm_s <= 0
        or cfg.enable_cooldown_s <= 0
        or cfg.button_hold_s <= 0
        or cfg.zero_settle_s < 0
        or cfg.zero_timeout_s <= cfg.zero_settle_s
    ):
        raise ValueError("rates, age limits, button hold, and enable timing must be positive")
    logging.basicConfig(
        level=logging.WARNING if cfg.quiet else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    log = logging.getLogger("umi-yam-source")

    vive = VivePacketReceiver(
        {cfg.left_serial: "left", cfg.right_serial: "right"},
        host=cfg.vive_bind,
        port=cfg.vive_port,
    )
    workers: dict[str, WidthWorker] = {}
    if not cfg.arm_only:
        from umi_teleop.camera import LocalGstCamera
        from umi_teleop.gripper import GripperConfig, GripperGeometry, detector_from_config

        cameras = {
            "left": LocalGstCamera(
                cfg.left_device, cfg.width, cfg.height, cfg.camera_fps, cfg.exposure
            ),
            "right": LocalGstCamera(
                cfg.right_device, cfg.width, cfg.height, cfg.camera_fps, cfg.exposure
            ),
        }
        gripper_cfgs = {
            "left": GripperConfig(
                geom=GripperGeometry(),
                roi_top_frac=cfg.roi_top_frac,
                calib_path=_calib(cfg.left_calib),
            ),
            "right": GripperConfig(
                geom=GripperGeometry(),
                roi_top_frac=cfg.roi_top_frac,
                calib_path=_calib(cfg.right_calib),
            ),
        }
        tag_ids = {"left": cfg.left_tag_ids, "right": cfg.right_tag_ids}
        workers = {
            name: WidthWorker(
                camera,
                detector_from_config(
                    gripper_cfgs[name], cfg.width, cfg.height, target_ids=tag_ids[name]
                ),
                detect_hz=cfg.detect_hz,
            )
            for name, camera in cameras.items()
        }
    buttons = (
        GpioButtons(
            record_pin=cfg.record_pin,
            quit_pin=cfg.quit_pin,
            hold_s=cfg.button_hold_s,
        )
        if cfg.buttons == "gpio"
        else KeyboardButtons()
    )
    state = ButtonState()
    enable_guard = EnableRequestGuard(
        delay_s=cfg.enable_confirm_s,
        cooldown_s=cfg.enable_cooldown_s,
    )
    zero_guard = ZeroRequestGuard(
        settle_s=cfg.zero_settle_s,
        timeout_s=cfg.zero_timeout_s,
    )
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (cfg.target_host, cfg.target_port)
    period = 1.0 / cfg.send_hz
    seq = 0
    quit_until: float | None = None
    last_status = 0.0

    log.info(
        "source session=%s; Vive UDP :%d -> YAM %s:%d; left=%s right=%s arm_only=%s",
        state.session_id,
        cfg.vive_port,
        cfg.target_host,
        cfg.target_port,
        cfg.left_serial,
        cfg.right_serial,
        cfg.arm_only,
    )
    if cfg.buttons == "keyboard":
        log.info("controls: type Z=ZERO, E=start, S=stop, Q=quit, then Enter")
    else:
        log.info(
            "controls: record tap=enable/disable, record hold=cancel, quit tap=ZERO, hold=quit"
        )

    try:
        vive.start()
        for worker in workers.values():
            worker.start()
        next_send = time.monotonic()
        while True:
            now = time.monotonic()
            while not state.quitting and (event := buttons.poll()) is not None:
                if event is Event.ZERO:
                    enable_guard.force_block(now)
                    action = zero_guard.request(state, now)
                elif zero_guard.pending and event in (Event.START, Event.TOGGLE):
                    action = "enable ignored while zero is pending"
                else:
                    if event in (Event.STOP, Event.CANCEL, Event.QUIT):
                        zero_guard.cancel()
                    action = enable_guard.handle(state, event, now)
                if cfg.quiet:
                    print(f"{event.name}: {action}", flush=True)
                else:
                    log.info(
                        "button=%s action=%s enabled=%s enable_pending=%s zero_pending=%s "
                        "zero_id=%d toggle_id=%d cancel_id=%d quit_id=%d",
                        event.name,
                        action,
                        state.requested_enabled,
                        enable_guard.pending,
                        zero_guard.pending,
                        state.zero_id,
                        state.toggle_id,
                        state.cancel_id,
                        state.quit_id,
                    )
                if state.quitting and quit_until is None:
                    quit_until = time.monotonic() + cfg.quit_burst_s
                now = time.monotonic()

            poses = vive.snapshot(now)
            poses_fresh = all(
                name in poses and poses[name]["age_s"] <= cfg.max_pose_age_s for name in ARM_NAMES
            )
            if state.requested_enabled and any(
                name not in poses or poses[name]["age_s"] > cfg.max_pose_age_s for name in ARM_NAMES
            ):
                state.safety_disarm()
                enable_guard.force_block(now)
                log.warning(
                    "tracking stale while enabled; source disarmed; %s",
                    _tracking_status(
                        poses,
                        {"left": cfg.left_serial, "right": cfg.right_serial},
                    ),
                )
            widths = {
                name: (
                    workers[name].snapshot(now)
                    if name in workers
                    else {"width_mm": None, "age_s": None}
                )
                for name in ARM_NAMES
            }
            if (zero_action := zero_guard.update(state, now, poses_fresh=poses_fresh)) is not None:
                if cfg.quiet:
                    print(zero_action, flush=True)
                else:
                    log.info(
                        "%s enabled=%s zero_id=%d",
                        zero_action,
                        state.requested_enabled,
                        state.zero_id,
                    )
            widths_fresh = cfg.arm_only or all(
                widths[name]["width_mm"] is not None
                and widths[name]["age_s"] <= cfg.max_width_age_s
                for name in ARM_NAMES
            )
            inputs_fresh = poses_fresh and widths_fresh
            if (
                enable_action := enable_guard.update(
                    state,
                    now,
                    inputs_fresh=inputs_fresh,
                    stale_description=(
                        "pose telemetry" if cfg.arm_only else "pose or width telemetry"
                    ),
                )
            ) is not None:
                if cfg.quiet:
                    print(enable_action, flush=True)
                else:
                    log.info(
                        "%s enabled=%s toggle_id=%d cancel_id=%d",
                        enable_action,
                        state.requested_enabled,
                        state.toggle_id,
                        state.cancel_id,
                    )
            packet = state.packet(seq, poses, widths)
            sock.sendto(encode_teleop_packet(packet), target)
            seq += 1

            if not cfg.quiet and now - last_status >= 1.0:
                log.info(
                    "enabled=%s enable_pending=%s zero_pending=%s pose_age[L=%s R=%s] "
                    "width_mm[L=%s R=%s] width_age[L=%s R=%s] bad_vive=%d",
                    state.requested_enabled,
                    enable_guard.pending,
                    zero_guard.pending,
                    _status_value(poses.get("left", {}), "age_s"),
                    _status_value(poses.get("right", {}), "age_s"),
                    _status_value(widths["left"], "width_mm"),
                    _status_value(widths["right"], "width_mm"),
                    _status_value(widths["left"], "age_s"),
                    _status_value(widths["right"], "age_s"),
                    vive.bad_packet_count,
                )
                last_status = now

            if quit_until is not None and now >= quit_until:
                break
            next_send += period
            time.sleep(max(0.0, next_send - time.monotonic()))
    finally:
        for worker in workers.values():
            worker.stop()
        vive.stop()
        close = getattr(buttons, "close", None)
        if callable(close):
            close()
        sock.close()


def main() -> None:
    try:
        run(tyro.cli(Config))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

"""Fuse local Vive poses with Pi ArUco/GPIO telemetry for the YAM controller.

This internal aq-yam process receives native libsurvive UDP and versioned Pi sensor packets, then
emits validated controller telemetry over loopback. It never opens CAN or commands a robot.
"""

from __future__ import annotations

import logging
import socket
import sys
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import tyro

from umi_teleop.buttons import GpioButtons, KeyboardButtons
from umi_teleop.debug_control import DebugControlReceiver, DebugControlRequest
from umi_teleop.pi_protocol import EVENT_NAMES, PiPacketReceiver
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
    """Native libsurvive sender destination port."""
    pi_bind: str = "0.0.0.0"
    pi_port: int = 9139
    pi_expected_host: str | None = "right-umi-vive-teleop.local"
    pi_heartbeat_timeout_s: float = 0.5
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
    buttons: Literal["pi", "gpio", "keyboard"] = "pi"
    record_pin: int = 17
    quit_pin: int = 27
    quit_burst_s: float = 0.75
    arm_only: bool = False
    """Skip gripper cameras and require only fresh Vive poses for ZERO and enable."""
    quiet: bool = False
    """Show only controls, state changes, warnings, and errors."""
    preflight_only: bool = False
    preflight_timeout_s: float = 20.0
    debug_control_port: int | None = None
    """Opt-in loopback command port for diagnostic ZERO/start/stop/cancel/quit injection."""


class _PiEventConsumer:
    """Convert repeated Pi counters into local events without replaying old history."""

    def __init__(self) -> None:
        self.session_id: str | None = None
        self.counters = {name: 0 for name in EVENT_NAMES}

    def update(self, packet: dict[str, Any]) -> tuple[bool, list[Event]]:
        session_changed = packet["session_id"] != self.session_id
        if session_changed:
            self.session_id = packet["session_id"]
            self.counters = dict(packet["events"])
            return True, []

        changed = {
            name: max(0, packet["events"][name] - self.counters[name]) for name in EVENT_NAMES
        }
        self.counters = {
            name: max(self.counters[name], packet["events"][name]) for name in EVENT_NAMES
        }
        if changed["quit"]:
            return False, [Event.QUIT]
        if changed["cancel"]:
            return False, [Event.CANCEL]
        if changed["zero"]:
            return False, [Event.ZERO]
        if changed["toggle"] > 1:
            # The exact ordering was lost during a network gap. Disarm instead of guessing.
            return False, [Event.CANCEL]
        return False, [Event.TOGGLE] * changed["toggle"]


@dataclass
class _ViveContinuityGuard:
    ids: dict[str, int] | None = None

    def update(self, poses: dict[str, dict[str, Any]]) -> bool:
        if set(poses) != set(ARM_NAMES):
            return False
        values = {name: poses[name].get("continuity_id") for name in ARM_NAMES}
        if any(value is None for value in values.values()):
            return False
        current = {name: int(values[name]) for name in ARM_NAMES}
        if self.ids is None:
            self.ids = current
            return False
        changed = current != self.ids
        self.ids = current
        return changed


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
        or cfg.pi_heartbeat_timeout_s <= 0
        or cfg.preflight_timeout_s <= 0
    ):
        raise ValueError("rates, age limits, button hold, and enable timing must be positive")
    if cfg.debug_control_port is not None and not 1 <= cfg.debug_control_port <= 65_535:
        raise ValueError("debug-control port must be between 1 and 65535")
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
    pi_receiver = (
        PiPacketReceiver(
            cfg.pi_bind,
            cfg.pi_port,
            expected_host=cfg.pi_expected_host,
        )
        if cfg.buttons == "pi"
        else None
    )
    pi_events = _PiEventConsumer()
    debug_control = (
        DebugControlReceiver(cfg.debug_control_port) if cfg.debug_control_port is not None else None
    )
    vive_continuity = _ViveContinuityGuard()
    workers: dict[str, WidthWorker] = {}
    if not cfg.arm_only and cfg.buttons != "pi":
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
    buttons = None
    if cfg.buttons == "gpio":
        buttons = GpioButtons(
            record_pin=cfg.record_pin,
            quit_pin=cfg.quit_pin,
            hold_s=cfg.button_hold_s,
        )
    elif cfg.buttons == "keyboard":
        buttons = KeyboardButtons()
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
    startup_deadline = time.monotonic() + cfg.preflight_timeout_s
    inputs_ready_once = False

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
    elif cfg.buttons == "gpio":
        log.info(
            "controls: record tap=enable/disable, record hold=cancel, quit tap=ZERO, hold=quit"
        )
    else:
        log.info(
            "controls arrive from Pi %s:%d; record tap=start/stop, record hold=cancel, "
            "zero tap=ZERO, zero hold=quit",
            cfg.pi_expected_host or "any host",
            cfg.pi_port,
        )
    if debug_control is not None:
        print(
            f"DEBUG CONTROL enabled on loopback UDP :{cfg.debug_control_port}; "
            "use umi-teleop-control",
            flush=True,
        )

    try:
        vive.start()
        for worker in workers.values():
            worker.start()
        next_send = time.monotonic()
        while True:
            now = time.monotonic()
            pending_events: list[tuple[Event, DebugControlRequest | None]] = []
            pi_packet = None
            pi_age = None
            if pi_receiver is not None:
                pi_packet, pi_age = pi_receiver.poll()
                if pi_packet is not None:
                    session_changed, pi_pending_events = pi_events.update(pi_packet)
                    pending_events.extend((event, None) for event in pi_pending_events)
                    if session_changed and inputs_ready_once:
                        state = ButtonState(session_id=uuid.uuid4().hex)
                        enable_guard = EnableRequestGuard(
                            delay_s=cfg.enable_confirm_s,
                            cooldown_s=cfg.enable_cooldown_s,
                        )
                        zero_guard = ZeroRequestGuard(
                            settle_s=cfg.zero_settle_s,
                            timeout_s=cfg.zero_timeout_s,
                        )
                        log.warning("Pi source restarted; teleoperation zero was invalidated")
            elif buttons is not None:
                while (event := buttons.poll()) is not None:
                    pending_events.append((event, None))

            if debug_control is not None:
                for request in debug_control.poll():
                    if request.event is None:
                        debug_control.reply(
                            request,
                            action="status",
                            requested_enabled=state.requested_enabled,
                            zero_id=state.zero_id,
                            toggle_id=state.toggle_id,
                            cancel_id=state.cancel_id,
                            quit_id=state.quit_id,
                        )
                    else:
                        pending_events.append((request.event, request))

            for event, debug_request in pending_events:
                if state.quitting:
                    break
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
                if debug_request is not None:
                    assert debug_control is not None
                    debug_control.reply(
                        debug_request,
                        action=action,
                        requested_enabled=state.requested_enabled,
                        zero_id=state.zero_id,
                        toggle_id=state.toggle_id,
                        cancel_id=state.cancel_id,
                        quit_id=state.quit_id,
                    )
                if state.quitting and quit_until is None:
                    quit_until = time.monotonic() + cfg.quit_burst_s
                now = time.monotonic()

            poses = vive.snapshot(now)
            if vive_continuity.update(poses):
                state = ButtonState(session_id=uuid.uuid4().hex)
                enable_guard = EnableRequestGuard(
                    delay_s=cfg.enable_confirm_s,
                    cooldown_s=cfg.enable_cooldown_s,
                )
                zero_guard = ZeroRequestGuard(
                    settle_s=cfg.zero_settle_s,
                    timeout_s=cfg.zero_timeout_s,
                )
                log.warning("Vive tracking continuity changed; teleoperation zero was invalidated")
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
            if pi_receiver is not None:
                widths = {}
                for name in ARM_NAMES:
                    arm = {} if pi_packet is None else pi_packet["arms"][name]
                    usable = bool(
                        pi_packet is not None
                        and pi_age is not None
                        and pi_age <= cfg.pi_heartbeat_timeout_s
                        and arm.get("camera_ok")
                        and arm.get("tags_visible")
                        and arm.get("width_mm") is not None
                        and arm.get("width_age_s") is not None
                    )
                    widths[name] = {
                        "width_mm": arm.get("width_mm") if usable else None,
                        "age_s": (float(arm["width_age_s"]) + float(pi_age) if usable else None),
                    }
            else:
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
            pi_fresh = pi_receiver is None or (
                pi_packet is not None
                and pi_age is not None
                and pi_age <= cfg.pi_heartbeat_timeout_s
            )
            inputs_fresh = inputs_fresh and pi_fresh
            if state.requested_enabled and not inputs_fresh:
                state.safety_disarm()
                enable_guard.force_block(now)
                log.warning("sensor input stale while enabled; source disarmed")
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

            if inputs_fresh:
                if not inputs_ready_once:
                    inputs_ready_once = True
                    print(
                        "SOURCE PREFLIGHT OK: Vive poses and Pi gripper telemetry are fresh",
                        flush=True,
                    )
                if cfg.preflight_only:
                    return
            elif not inputs_ready_once and now >= startup_deadline:
                missing = []
                if not poses_fresh:
                    missing.append("both configured Vive tracker poses")
                if not pi_fresh:
                    missing.append("Pi heartbeat")
                elif not widths_fresh:
                    missing.append("both Pi cameras and ArUco tag pairs")
                raise RuntimeError(
                    f"source preflight timed out after {cfg.preflight_timeout_s:g}s waiting for "
                    + ", ".join(missing)
                )

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
        if pi_receiver is not None:
            pi_receiver.close()
        if debug_control is not None:
            debug_control.close()
        sock.close()


def main() -> None:
    try:
        run(tyro.cli(Config))
    except KeyboardInterrupt:
        pass
    except (FileNotFoundError, OSError, RuntimeError, ValueError) as exc:
        print(f"SOURCE ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == "__main__":
    main()

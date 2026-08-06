"""Merge Lambda LiDAR poses and per-UMI widths/buttons into the existing YAM protocol."""

from __future__ import annotations

import logging
import socket
import time
from dataclasses import dataclass, field

import tyro

from umi_teleop.lidar import LidarArmReceiver, LidarPoseReceiver
from umi_teleop.protocol import (
    ARM_NAMES,
    ButtonState,
    EnableRequestGuard,
    ZeroRequestGuard,
    encode_teleop_packet,
)
from umi_teleop.types import Event


@dataclass
class _SourceCounters:
    session_id: str = ""
    requested_enabled: bool = False
    zero_id: int = 0
    toggle_id: int = 0
    cancel_id: int = 0
    quit_id: int = 0


@dataclass
class Config:
    """LiDAR teleop merger settings."""

    target_host: str = "127.0.0.1"
    target_port: int = 9140
    pose_bind: str = "0.0.0.0"
    pose_port: int = 9142
    arm_bind: str = "0.0.0.0"
    arm_port: int = 9143
    send_hz: float = 30.0
    max_pose_age_s: float = 0.25
    max_width_age_s: float = 0.5
    enable_confirm_s: float = 3.0
    enable_cooldown_s: float = 5.0
    zero_settle_s: float = 0.4
    zero_timeout_s: float = 4.0
    source_state: dict[str, _SourceCounters] = field(default_factory=dict, init=False)


def _events_from_source(packet: dict, previous: _SourceCounters) -> list[Event]:
    events: list[Event] = []
    if packet["session_id"] != previous.session_id:
        previous.session_id = packet["session_id"]
        previous.requested_enabled = False
        previous.zero_id = 0
        previous.toggle_id = 0
        previous.cancel_id = 0
        previous.quit_id = 0
    for _ in range(max(0, packet["zero_id"] - previous.zero_id)):
        events.append(Event.ZERO)
    for _ in range(max(0, packet["cancel_id"] - previous.cancel_id)):
        events.append(Event.CANCEL)
    for _ in range(max(0, packet["quit_id"] - previous.quit_id)):
        events.append(Event.QUIT)
    if packet["toggle_id"] > previous.toggle_id:
        event = Event.START if packet["requested_enabled"] else Event.STOP
        events.extend([event] * (packet["toggle_id"] - previous.toggle_id))
    previous.requested_enabled = packet["requested_enabled"]
    previous.zero_id = max(previous.zero_id, packet["zero_id"])
    previous.toggle_id = max(previous.toggle_id, packet["toggle_id"])
    previous.cancel_id = max(previous.cancel_id, packet["cancel_id"])
    previous.quit_id = max(previous.quit_id, packet["quit_id"])
    return events


def _fresh_poses(poses: dict[str, dict], limit: float) -> bool:
    return all(name in poses and poses[name]["age_s"] <= limit for name in ARM_NAMES)


def _fresh_inputs(
    poses: dict[str, dict],
    widths: dict[str, dict],
    *,
    pose_limit: float,
    width_limit: float,
) -> bool:
    return all(
        name in poses
        and poses[name]["age_s"] <= pose_limit
        and widths[name]["width_mm"] is not None
        and widths[name]["age_s"] <= width_limit
        for name in ARM_NAMES
    )


def run(cfg: Config) -> None:
    if cfg.send_hz <= 0:
        raise ValueError("--send-hz must be positive")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    log = logging.getLogger("umi-lidar-merge")
    poses = LidarPoseReceiver(cfg.pose_bind, cfg.pose_port)
    arms = LidarArmReceiver(cfg.arm_bind, cfg.arm_port)
    state = ButtonState()
    enable_guard = EnableRequestGuard(cfg.enable_confirm_s, cfg.enable_cooldown_s)
    zero_guard = ZeroRequestGuard(cfg.zero_settle_s, cfg.zero_timeout_s)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (cfg.target_host, cfg.target_port)
    period = 1.0 / cfg.send_hz
    seq = 0
    last_status = 0.0
    log.info(
        "merge session=%s; poses :%d arms :%d -> YAM %s:%d",
        state.session_id,
        cfg.pose_port,
        cfg.arm_port,
        cfg.target_host,
        cfg.target_port,
    )
    try:
        poses.start()
        arms.start()
        while True:
            started = time.monotonic()
            now = time.monotonic()
            pose_snapshot = poses.snapshot(now)
            width_snapshot = arms.widths(now)
            for source_id, packet in sorted(arms.latest_packets().items()):
                previous = cfg.source_state.setdefault(source_id, _SourceCounters())
                for event in _events_from_source(packet, previous):
                    if event is Event.ZERO:
                        enable_guard.force_block(now)
                        action = zero_guard.request(state, now)
                    elif zero_guard.pending and event in (Event.START, Event.TOGGLE):
                        action = "enable ignored while zero is pending"
                    else:
                        if event in (Event.STOP, Event.CANCEL, Event.QUIT):
                            zero_guard.cancel()
                        action = enable_guard.handle(state, event, now)
                    log.info(
                        "source=%s button=%s action=%s enabled=%s enable_pending=%s "
                        "zero_pending=%s zero_id=%d toggle_id=%d cancel_id=%d quit_id=%d",
                        source_id,
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
            if state.requested_enabled and not _fresh_poses(pose_snapshot, cfg.max_pose_age_s):
                state.safety_disarm()
                enable_guard.force_block(now)
                log.warning("LiDAR pose stale while enabled; merger disarmed")
            if (
                zero_action := zero_guard.update(
                    state,
                    now,
                    poses_fresh=_fresh_poses(pose_snapshot, cfg.max_pose_age_s),
                )
            ) is not None:
                log.info(
                    "%s enabled=%s zero_id=%d", zero_action, state.requested_enabled, state.zero_id
                )
            if (
                enable_action := enable_guard.update(
                    state,
                    now,
                    inputs_fresh=_fresh_inputs(
                        pose_snapshot,
                        width_snapshot,
                        pose_limit=cfg.max_pose_age_s,
                        width_limit=cfg.max_width_age_s,
                    ),
                )
            ) is not None:
                log.info(
                    "%s enabled=%s toggle_id=%d cancel_id=%d",
                    enable_action,
                    state.requested_enabled,
                    state.toggle_id,
                    state.cancel_id,
                )
            packet = state.packet(seq, pose_snapshot, width_snapshot)
            sock.sendto(encode_teleop_packet(packet), target)
            seq += 1
            if now - last_status >= 1.0:
                log.info(
                    "enabled=%s enable_pending=%s zero_pending=%s pose_age[L=%s R=%s] "
                    "width_mm[L=%s R=%s] width_age[L=%s R=%s] bad_pose=%d bad_arm=%d",
                    state.requested_enabled,
                    enable_guard.pending,
                    zero_guard.pending,
                    _fmt(pose_snapshot.get("left", {}), "age_s"),
                    _fmt(pose_snapshot.get("right", {}), "age_s"),
                    _fmt(width_snapshot["left"], "width_mm"),
                    _fmt(width_snapshot["right"], "width_mm"),
                    _fmt(width_snapshot["left"], "age_s"),
                    _fmt(width_snapshot["right"], "age_s"),
                    poses.bad_packet_count,
                    arms.bad_packet_count,
                )
                last_status = now
            if state.quitting:
                break
            time.sleep(max(0.0, period - (time.monotonic() - started)))
    finally:
        poses.stop()
        arms.stop()


def _fmt(value: dict, key: str) -> str:
    item = value.get(key)
    return "--" if item is None else f"{float(item):.3f}"


def main() -> None:
    run(tyro.cli(Config))


if __name__ == "__main__":
    main()

"""Stream live GLIM poses from Lambda containers to a teleop merger."""

from __future__ import annotations

import json
import logging
import socket
import subprocess
import threading
import time
from base64 import b64encode
from dataclasses import dataclass

import tyro

from umi_teleop.lidar import encode_lidar_pose_packet, glim_odom_to_umi_pose

POSE_INNER = (
    "source /opt/ros/jazzy/setup.bash; "
    "source /ws/install/setup.bash; "
    "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; "
    "python3 /pose_sub.py"
)


@dataclass(frozen=True)
class ArmContainer:
    arm: str
    container: str
    serial: str


@dataclass
class Config:
    """Lambda live-GLIM pose sender settings."""

    target_host: str = ""
    """Host running umi-lidar-merge-source, usually the YAM Pi or operator workstation."""
    target_port: int = 9142
    left_container: str = ""
    right_container: str = ""
    left_serial: str = "left-lidar"
    right_serial: str = "right-lidar"
    hz: float = 30.0
    docker: str = "docker"
    include_tcp_offset: bool = False
    """Debug only: pre-shift pose translation to the TCP. Normal teleop leaves this false."""
    stdout_packets: bool = False
    """Write base64-encoded pose datagrams to stdout instead of sending UDP."""


class PoseStream:
    def __init__(self, docker: str, item: ArmContainer, include_tcp_offset: bool):
        self.docker = docker
        self.item = item
        self.include_tcp_offset = include_tcp_offset
        self.lock = threading.Lock()
        self.pose: list[float] | None = None
        self.source_time_s: float | None = None
        self.samples = 0
        self.errors = 0
        self.message = "starting"
        self.proc: subprocess.Popen[str] | None = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self.thread.start()

    def _run_once(self) -> None:
        cmd = [self.docker, "exec", "-i", self.item.container, "bash", "-lc", POSE_INNER]
        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        assert self.proc.stdout is not None
        for line in self.proc.stdout:
            try:
                raw = json.loads(line)
            except ValueError:
                continue
            if "info" in raw:
                self.message = str(raw["info"])
                continue
            try:
                pose = glim_odom_to_umi_pose(
                    [raw["x"], raw["y"], raw["z"]],
                    [
                        raw.get("qx", 0.0),
                        raw.get("qy", 0.0),
                        raw.get("qz", 0.0),
                        raw.get("qw", 1.0),
                    ],
                    tip_offset_m=(-0.10479, 0.0, 0.22244) if self.include_tcp_offset else None,
                )
                stamp = float(raw["t"])
            except (KeyError, TypeError, ValueError):
                self.errors += 1
                continue
            with self.lock:
                self.pose = pose
                self.source_time_s = stamp
                self.samples += 1
                self.message = "streaming"

    def _run(self) -> None:
        while True:
            try:
                self._run_once()
            except Exception as exc:  # noqa: BLE001 - keep a live robot telemetry path resilient
                self.errors += 1
                self.message = f"stream error: {exc}"
            time.sleep(1.0)

    def snapshot(self) -> dict | None:
        with self.lock:
            if self.pose is None or self.source_time_s is None:
                return None
            return {
                "serial": self.item.serial,
                "pose": list(self.pose),
                "source_time_s": self.source_time_s,
            }


def _arm_containers(cfg: Config) -> list[ArmContainer]:
    items = []
    if cfg.left_container:
        items.append(ArmContainer("left", cfg.left_container, cfg.left_serial))
    if cfg.right_container:
        items.append(ArmContainer("right", cfg.right_container, cfg.right_serial))
    if not items:
        raise ValueError("at least one of --left-container or --right-container is required")
    return items


def run(cfg: Config) -> None:
    if cfg.hz <= 0:
        raise ValueError("--hz must be positive")
    if not cfg.stdout_packets and not cfg.target_host:
        raise ValueError("--target-host is required unless --stdout-packets is set")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    log = logging.getLogger("umi-lidar-lambda")
    streams = [
        PoseStream(cfg.docker, item, cfg.include_tcp_offset) for item in _arm_containers(cfg)
    ]
    for stream in streams:
        stream.start()
    sock = None if cfg.stdout_packets else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (cfg.target_host, cfg.target_port)
    seq = 0
    period = 1.0 / cfg.hz
    last_status = 0.0
    if cfg.stdout_packets:
        log.info("writing live GLIM pose datagrams to stdout")
    else:
        log.info("sending live GLIM poses to %s:%d", cfg.target_host, cfg.target_port)
    while True:
        started = time.monotonic()
        arms = {}
        for stream in streams:
            snap = stream.snapshot()
            if snap is not None:
                arms[stream.item.arm] = snap
        if arms:
            packet = {
                "version": 1,
                "source": "lambda-glim",
                "seq": seq,
                "arms": arms,
            }
            data = encode_lidar_pose_packet(packet)
            if cfg.stdout_packets:
                print(b64encode(data).decode("ascii"), flush=True)
            else:
                assert sock is not None
                sock.sendto(data, target)
            seq += 1
        now = time.monotonic()
        if now - last_status >= 1.0:
            log.info(
                "seq=%d arms=%s %s",
                seq,
                ",".join(sorted(arms)) or "--",
                " ".join(
                    f"{s.item.arm}:{s.samples} samples/{s.errors} errors/{s.message}"
                    for s in streams
                ),
            )
            last_status = now
        time.sleep(max(0.0, period - (time.monotonic() - started)))


def main() -> None:
    run(tyro.cli(Config))


if __name__ == "__main__":
    main()

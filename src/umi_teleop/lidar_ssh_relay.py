"""Relay Lambda GLIM pose datagrams over SSH stdout to LAN UDP."""

from __future__ import annotations

import argparse
import base64
import logging
import shlex
import socket
import subprocess
import time
from dataclasses import dataclass


@dataclass
class Config:
    lambda_host: str = "ubuntu@192.222.52.98"
    lambda_repo: str = "/home/ubuntu/umi-teleop"
    target_host: str = "10.103.3.85"
    target_port: int = 9142
    left_container: str = "glim-10-103-0-84"
    right_container: str = "glim-10-103-0-80"
    left_serial: str = "10.103.0.84"
    right_serial: str = "10.103.0.80"


def _remote_cmd(cfg: Config) -> str:
    args = [
        f"cd {shlex.quote(cfg.lambda_repo)}",
        "&&",
        f"PYTHONPATH={shlex.quote(cfg.lambda_repo + '/src')}",
        ".venv/bin/python",
        "-m",
        "umi_teleop.lambda_lidar",
        "--stdout-packets",
        "--left-container",
        shlex.quote(cfg.left_container),
        "--right-container",
        shlex.quote(cfg.right_container),
        "--left-serial",
        shlex.quote(cfg.left_serial),
        "--right-serial",
        shlex.quote(cfg.right_serial),
    ]
    return " ".join(args)


def run(cfg: Config) -> None:
    if cfg.target_port <= 0:
        raise ValueError("--target-port must be positive")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    log = logging.getLogger("umi-lidar-ssh-relay")
    cmd = [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ServerAliveInterval=15",
        "-o",
        "ServerAliveCountMax=3",
        cfg.lambda_host,
        _remote_cmd(cfg),
    ]
    log.info("starting Lambda stdout pose stream from %s", cfg.lambda_host)
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
    assert proc.stdout is not None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (cfg.target_host, cfg.target_port)
    sent = 0
    last_status = 0.0
    for line in proc.stdout:
        line = line.strip()
        if not line:
            continue
        try:
            data = base64.b64decode(line, validate=True)
        except ValueError as exc:
            log.warning("skipped bad packet line: %s", exc)
            continue
        sock.sendto(data, target)
        sent += 1
        now = time.monotonic()
        if now - last_status >= 1.0:
            log.info("relayed %d pose datagrams to %s:%d", sent, cfg.target_host, cfg.target_port)
            last_status = now
    raise RuntimeError(f"Lambda pose stream exited with code {proc.wait()}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--lambda-host", default=Config.lambda_host)
    parser.add_argument("--lambda-repo", default=Config.lambda_repo)
    parser.add_argument("--target-host", default=Config.target_host)
    parser.add_argument("--target-port", type=int, default=Config.target_port)
    parser.add_argument("--left-container", default=Config.left_container)
    parser.add_argument("--right-container", default=Config.right_container)
    parser.add_argument("--left-serial", default=Config.left_serial)
    parser.add_argument("--right-serial", default=Config.right_serial)
    run(Config(**vars(parser.parse_args())))


if __name__ == "__main__":
    main()

from __future__ import annotations

import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest


def free_udp_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


@pytest.mark.integration
@pytest.mark.skipif(sys.platform != "linux", reason="native launcher is Linux-only")
def test_pi_source_and_teleop_preflight_run_as_two_processes() -> None:
    binaries = Path(sys.executable).parent
    pi_port = free_udp_port()
    vive_port = free_udp_port()
    controller_port = free_udp_port()
    pi = subprocess.Popen(
        [
            str(binaries / "umi-pi-source"),
            "--network.target-host",
            "127.0.0.1",
            "--network.target-port",
            str(pi_port),
            "--simulate",
            "--max-packets",
            "900",
            "--quiet",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        time.sleep(0.1)
        completed = subprocess.run(
            [
                str(binaries / "umi-teleop"),
                "--pi.source-host",
                "127.0.0.1",
                "--pi.bind",
                "127.0.0.1",
                "--pi.port",
                str(pi_port),
                "--tracking.port",
                str(vive_port),
                "--tracking.simulate",
                "--control.telemetry-port",
                str(controller_port),
                "--simulate",
                "--preflight-only",
                "--quiet",
            ],
            check=False,
            text=True,
            capture_output=True,
            timeout=30.0,
        )
        output = completed.stdout + completed.stderr
        assert completed.returncode == 0, output
        assert "YAM PREFLIGHT OK" in output
        assert "SOURCE PREFLIGHT OK" in output
    finally:
        pi.terminate()
        try:
            pi.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            pi.kill()
            pi.wait(timeout=3.0)

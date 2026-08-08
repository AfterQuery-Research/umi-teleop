"""Shared i2rt lifecycle helpers, isolated from the rest of the application."""

from __future__ import annotations

import threading
from typing import Any

I2RT_CONTROL_THREAD = "_set_torques_and_update_state"


def new_control_threads(
    existing_threads: set[threading.Thread],
) -> tuple[threading.Thread, ...]:
    """Find the low-level CAN workers created by one i2rt robot."""
    return tuple(
        thread
        for thread in threading.enumerate()
        if thread not in existing_threads and I2RT_CONTROL_THREAD in thread.name
    )


def close_i2rt_robot(
    robot: Any,
    control_threads: tuple[threading.Thread, ...],
) -> None:
    """Stop i2rt workers before closing their SocketCAN file descriptor."""
    robot._stop_event.set()
    robot._server_thread.join(timeout=2.0)

    robot.motor_chain.running = False
    for thread in control_threads:
        thread.join(timeout=2.0)

    worker_threads = (robot._server_thread, *control_threads)
    stuck_threads = [thread.name for thread in worker_threads if thread.is_alive()]
    robot.close()
    if stuck_threads:
        names = ", ".join(stuck_threads)
        raise RuntimeError(f"i2rt CAN control thread did not stop cleanly: {names}")

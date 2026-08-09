from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class ButtonSnapshot:
    """Cumulative, loss-tolerant control state sent in every UDP packet."""

    pressed: bool
    engage_count: int
    quit_count: int


class ButtonEventState:
    """Classify a debounced release as engage and a hold as quit exactly once."""

    def __init__(self, *, pressed: bool = False) -> None:
        self._lock = threading.Lock()
        self._pressed = pressed
        self._held = False
        self._armed = not pressed
        self._engage_count = 0
        self._quit_count = 0

    def press(self) -> None:
        with self._lock:
            if self._pressed:
                return
            self._pressed = True
            self._held = False

    def hold(self) -> None:
        with self._lock:
            if not self._pressed or self._held:
                return
            self._held = True
            if self._armed:
                self._quit_count += 1

    def release(self) -> None:
        with self._lock:
            if not self._pressed:
                return
            engage = self._armed and not self._held
            self._pressed = False
            self._held = False
            if not self._armed:
                self._armed = True
            elif engage:
                self._engage_count += 1

    def snapshot(self) -> ButtonSnapshot:
        with self._lock:
            return ButtonSnapshot(
                pressed=self._pressed,
                engage_count=self._engage_count,
                quit_count=self._quit_count,
            )


class ButtonDevice(Protocol):
    is_pressed: bool
    when_pressed: Callable[[], None] | None
    when_released: Callable[[], None] | None
    when_held: Callable[[], None] | None

    def close(self) -> None: ...


ButtonFactory = Callable[..., ButtonDevice]


class GpioButton:
    """One active-low BCM GPIO button backed by gpiozero on Raspberry Pi."""

    def __init__(
        self,
        *,
        line: int,
        debounce_seconds: float,
        hold_seconds: float,
        button_factory: ButtonFactory | None = None,
    ) -> None:
        if button_factory is None:
            try:
                from gpiozero import Button
            except ImportError as error:
                raise RuntimeError(
                    "GPIO button support is unavailable; install the Pi extra with "
                    "`uv sync --no-dev --extra pi`"
                ) from error
            button_factory = Button
        self._button = button_factory(
            line,
            pull_up=True,
            bounce_time=debounce_seconds,
            hold_time=hold_seconds,
            hold_repeat=False,
        )
        self._state = ButtonEventState(pressed=bool(self._button.is_pressed))
        self._button.when_pressed = self._state.press
        self._button.when_released = self._state.release
        self._button.when_held = self._state.hold

    def snapshot(self) -> ButtonSnapshot:
        return self._state.snapshot()

    def close(self) -> None:
        self._button.close()


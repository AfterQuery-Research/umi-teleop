"""Button input: emits START / STOP / TOGGLE / CANCEL / QUIT events."""

from __future__ import annotations

import select
import sys
import time
from abc import ABC, abstractmethod
from queue import Empty, Queue

from umi_teleop.types import Event


class Buttons(ABC):
    @abstractmethod
    def poll(self) -> Event | None:
        """Return the next event, or None if no button was pressed."""


class KeyboardButtons(Buttons):
    """Mock buttons for dev: type a key then Enter.

    e = toggle enable/disable, s = explicit start, c = cancel, z = zero pose, q = quit.
    POSIX-only (select() on stdin) — fine on Mac and the Pi.
    """

    KEYMAP = {
        "s": Event.START,
        "e": Event.TOGGLE,
        "c": Event.CANCEL,
        "z": Event.ZERO,
        "q": Event.QUIT,
    }

    def poll(self) -> Event | None:
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            return None
        line = sys.stdin.readline()
        if line == "":  # stdin closed
            return Event.QUIT
        return self.KEYMAP.get(line.strip().lower())


class GpioButtons(Buttons):
    """Two physical Pi buttons.

    The record button taps to toggle start/stop and holds to cancel the current
    recording. The quit button taps to re-zero the live Vive pose and holds to quit.
    Wire each button from its BCM GPIO pin to ground; gpiozero's default pull-up
    handles the idle state.
    """

    def __init__(
        self,
        record_pin: int = 17,
        quit_pin: int = 27,
        hold_s: float = 2.0,
        bounce_s: float = 0.05,
        toggle_debounce_s: float = 0.4,
    ):
        from gpiozero import Button

        self._events: Queue[Event] = Queue()
        self._record_held = False
        self._quit_held = False
        # Real button contact bounce once caused stop -> immediate restart. Suppress release
        # toggles inside a window shorter than any intentional human double tap.
        self._toggle_debounce_s = toggle_debounce_s
        self._last_toggle = 0.0
        # A bounce press after a hold release clears _record_held. Without this separate guard,
        # the following bounce release would restart immediately after CANCEL.
        self._hold_released_at = 0.0
        self._record = Button(record_pin, bounce_time=bounce_s, hold_time=hold_s)
        self._quit = Button(quit_pin, bounce_time=bounce_s, hold_time=hold_s)
        self._record.when_pressed = self._on_record_pressed
        self._record.when_held = self._on_record_held
        self._record.when_released = self._on_record_released
        self._quit.when_pressed = self._on_quit_pressed
        self._quit.when_held = self._on_quit_held
        self._quit.when_released = self._on_quit_released

    def _on_record_pressed(self) -> None:
        self._record_held = False

    def _on_record_held(self) -> None:
        self._record_held = True
        self._events.put(Event.CANCEL)

    def _on_record_released(self) -> None:
        if self._record_held:
            self._hold_released_at = time.monotonic()
            return
        now = time.monotonic()
        if now - self._hold_released_at < self._toggle_debounce_s:
            return
        if now - self._last_toggle < self._toggle_debounce_s:
            return
        self._last_toggle = now
        self._events.put(Event.TOGGLE)

    def _on_quit_pressed(self) -> None:
        self._quit_held = False

    def _on_quit_held(self) -> None:
        self._quit_held = True  # suppress the zero that release would otherwise emit
        self._events.put(Event.QUIT)

    def _on_quit_released(self) -> None:
        if not self._quit_held:  # tap (released before hold fired) -> zero the live pose
            self._events.put(Event.ZERO)

    def poll(self) -> Event | None:
        try:
            return self._events.get_nowait()
        except Empty:
            return None

    def close(self) -> None:
        self._record.close()
        self._quit.close()


class ScriptedButtons(Buttons):
    """Plays back a fixed sequence of events (used in tests)."""

    def __init__(self, events: list[Event | None]):
        self.events = list(events)

    def poll(self) -> Event | None:
        if not self.events:
            return Event.QUIT
        return self.events.pop(0)

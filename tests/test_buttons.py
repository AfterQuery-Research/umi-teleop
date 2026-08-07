from __future__ import annotations

import sys
from types import ModuleType

from umi_teleop import buttons as buttons_module
from umi_teleop.buttons import GpioButtons
from umi_teleop.types import Event


class FakeButton:
    instances: list[FakeButton] = []

    def __init__(self, pin: int, *, bounce_time: float, hold_time: float) -> None:
        self.pin = pin
        self.bounce_time = bounce_time
        self.hold_time = hold_time
        self.when_pressed = None
        self.when_held = None
        self.when_released = None
        self.closed = False
        self.instances.append(self)

    def close(self) -> None:
        self.closed = True


def install_fake_gpiozero(monkeypatch) -> None:
    FakeButton.instances = []
    module = ModuleType("gpiozero")
    module.Button = FakeButton  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "gpiozero", module)


def test_gpio_taps_holds_and_release_bounce_are_fail_closed(monkeypatch) -> None:
    install_fake_gpiozero(monkeypatch)
    now = [10.0]
    monkeypatch.setattr(buttons_module.time, "monotonic", lambda: now[0])
    buttons = GpioButtons(
        record_pin=17,
        quit_pin=27,
        hold_s=1.5,
        bounce_s=0.05,
        toggle_debounce_s=0.4,
    )
    record, zero_quit = FakeButton.instances

    record.when_pressed()
    record.when_released()
    assert buttons.poll() is Event.TOGGLE
    assert buttons.poll() is None

    now[0] = 11.0
    record.when_pressed()
    record.when_held()
    record.when_released()
    assert buttons.poll() is Event.CANCEL
    assert buttons.poll() is None

    now[0] = 11.1
    record.when_pressed()
    record.when_released()
    assert buttons.poll() is None

    now[0] = 12.0
    zero_quit.when_pressed()
    zero_quit.when_released()
    assert buttons.poll() is Event.ZERO

    zero_quit.when_pressed()
    zero_quit.when_held()
    zero_quit.when_released()
    assert buttons.poll() is Event.QUIT
    assert buttons.poll() is None

    buttons.close()
    assert record.closed and zero_quit.closed

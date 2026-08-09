from umi_teleop_fixed.gpio_button import ButtonEventState, GpioButton


def test_button_state_classifies_release_and_hold_without_double_action() -> None:
    state = ButtonEventState()

    state.press()
    state.release()
    assert state.snapshot().engage_count == 1

    state.press()
    state.hold()
    state.hold()
    state.release()
    snapshot = state.snapshot()
    assert snapshot.pressed is False
    assert snapshot.engage_count == 1
    assert snapshot.quit_count == 1


def test_button_held_during_startup_must_release_before_it_is_armed() -> None:
    state = ButtonEventState(pressed=True)

    state.hold()
    state.release()
    assert state.snapshot().engage_count == 0
    assert state.snapshot().quit_count == 0

    state.press()
    state.release()
    assert state.snapshot().engage_count == 1


def test_gpio_wrapper_uses_active_low_pullup_and_configured_timing() -> None:
    captured = {}

    class FakeButton:
        is_pressed = False
        when_pressed = None
        when_released = None
        when_held = None

        def close(self) -> None:
            captured["closed"] = True

    fake = FakeButton()

    def factory(line, **kwargs):
        captured.update(line=line, **kwargs)
        return fake

    button = GpioButton(
        line=16,
        debounce_seconds=0.05,
        hold_seconds=1.0,
        button_factory=factory,
    )
    assert captured == {
        "line": 16,
        "pull_up": True,
        "bounce_time": 0.05,
        "hold_time": 1.0,
        "hold_repeat": False,
    }

    fake.when_pressed()
    fake.when_released()
    assert button.snapshot().engage_count == 1
    button.close()
    assert captured["closed"] is True


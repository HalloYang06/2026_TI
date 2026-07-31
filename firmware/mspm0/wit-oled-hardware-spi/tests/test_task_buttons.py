from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]


def _function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_task_buttons_match_sw3_select_and_sw1_execute() -> None:
    syscfg = (PROJECT / "wit-oled-hardware-spi.syscfg").read_text(encoding="utf-8")
    key_header = (PROJECT / "Drivers" / "GRAY" / "key.h").read_text(
        encoding="utf-8"
    )
    key_source = (PROJECT / "Drivers" / "GRAY" / "key.c").read_text(
        encoding="utf-8"
    )

    assert 'GPIO7.associatedPins[1].assignedPin      = "8";' in syscfg
    assert 'GPIO7.associatedPins[2].assignedPin      = "9";' in syscfg
    assert "TASK_KEY_EXECUTE_PIN GPIO_KEY_KEY_1_PIN" in key_header
    assert "TASK_KEY_SELECT_PIN GPIO_KEY_KEY_3_PIN" in key_header
    assert "DL_GPIO_readPins(GPIO_KEY_PORT, pin) == 0U" in key_source


def test_task_menu_uses_separate_select_and_execute_events() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    body = _function_body(
        main,
        "static uint8_t select_car_task(void)\n{",
        "static int16_t approach_pwm",
    )

    assert "get_task_key_event()" in body
    assert "TASK_KEY_EVENT_SELECT" in body
    assert "TASK_KEY_EVENT_EXECUTE" in body
    assert "START_KEY_BUTTON_PIN" not in body
    assert "TASK_DOUBLE_CLICK_MS" not in main


def test_q2_execute_bypasses_distributed_mission_start() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    body = _function_body(
        main,
        "static uint8_t select_car_task(void)\n{",
        "static int16_t approach_pwm",
    )

    local_q2 = body.index("return HBALL_MISSION_Q2_FAST_LAP;")
    distributed_start = body.index(
        "hball_can_mission_menu_handle(mission_event, tick_ms)"
    )
    assert local_q2 < distributed_start

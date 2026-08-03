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
        "static void speed_calibration_test(void)\n{",
    )

    assert "get_task_key_event()" in body
    assert "TASK_KEY_EVENT_SELECT" in body
    assert "TASK_KEY_EVENT_EXECUTE" in body
    assert "START_KEY_BUTTON_PIN" not in body
    assert "TASK_DOUBLE_CLICK_MS" not in main


def test_sw1_start_is_latched_until_remote_mission_is_ready() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    body = _function_body(
        main,
        "static uint8_t select_car_task(void)\n{",
        "static void speed_calibration_test(void)\n{",
    )

    assert "#define HBALL_MISSION_START_LATCH_MS 5000U" in main
    assert "start_key_latched = true;" in body
    assert "hball_mission_client_ready(&snapshot, tick_ms)" in body
    assert "key_event = TASK_KEY_EVENT_EXECUTE;" in body
    assert 'telemetry_send_string("MISSION_SW1,LATCH\\r\\n")' in body
    assert 'telemetry_send_string("MISSION_SW1,START_ACCEPTED\\r\\n")' in body
    assert "remote_started = true;" in body
    assert "if (!abort_requested && !remote_started" in body

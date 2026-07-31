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


def test_task_menu_requires_release_before_power_on_key_arm() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    body = _function_body(
        main,
        "static uint8_t select_car_task(void)\n{",
        "static int16_t approach_pwm",
    )

    release_gate = body.index(
        "DL_GPIO_readPins(GPIO_KEY_PORT, TASK_KEY_EXECUTE_PIN) == 0U"
    )
    event_poll = body.index("get_task_key_event()")
    assert release_gate < event_poll


def test_q2_keeps_tested_c94_tracking_parameters() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    q2 = main.split(
        "if (selected_task == HBALL_MISSION_Q2_FAST_LAP)", 1
    )[1].split(
        "else if (selected_task == HBALL_MISSION_Q4_A_TO_B)", 1
    )[0]

    assert "base_speed = 44;" in q2
    assert "max_speed = 65;" in q2
    assert "recovery_inner_speed = 14;" in q2
    assert "weighted_position_kp = 0.50f;" in q2
    assert "steering_limit = 20;" in q2
    assert "steering_slew_step = 2;" in q2
    assert "run_timeout_ms = 35000U;" in q2
    assert "TRACK_PHASE_CURVE" not in main

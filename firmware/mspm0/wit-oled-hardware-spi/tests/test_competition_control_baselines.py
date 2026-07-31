from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]


def _function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_q2_verified_tracking_parameters_and_cadences_are_pinned() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    lap = _function_body(
        main,
        "static void lap_test_once(void)\n{",
        "void TIMER_0_INST_IRQHandler(void)",
    )
    q2 = lap[
        lap.index("if (selected_task == HBALL_MISSION_Q2_FAST_LAP)") :
        lap.index("else if (selected_task == HBALL_MISSION_Q4_A_TO_B)")
    ]

    for statement in (
        "base_speed = 63;",
        "max_speed = 80;",
        "recovery_inner_speed = 24;",
        "recovery_outer_speed = 52;",
        "weighted_position_kp = 0.65f;",
        "steering_limit = 28;",
        "steering_slew_step = 5;",
        "finish_line_min_run_ms = 18000U;",
        "run_timeout_ms = 0U;",
    ):
        assert statement in q2

    assert "const uint32_t speed_control_period_ms = 100U;" in lap
    assert "LEFT.Kp = 0.18f;" in lap
    assert "LEFT.Ki = 0.005f;" in lap
    assert "RIGHT.Kp = 0.18f;" in lap
    assert "RIGHT.Ki = 0.005f;" in lap
    assert "competition_runtime_wait_ms(10U);" in lap


def test_q4_keeps_verified_direct_start_without_unverified_ramp() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    lap = _function_body(
        main,
        "static void lap_test_once(void)\n{",
        "void TIMER_0_INST_IRQHandler(void)",
    )
    q4 = lap[
        lap.index("else if (selected_task == HBALL_MISSION_Q4_A_TO_B)") :
        lap.index("else\n    {", lap.index("else if (selected_task == HBALL_MISSION_Q4_A_TO_B)"))
    ]

    assert "selected_task = CAR_TASK_TIMED_RUN;" in q4
    assert "run_timeout_ms = 7800U;" in q4
    assert "task2_start_speed" not in lap
    assert "task2_start_ramp_ms" not in lap
    assert "if (selected_task == CAR_TASK_STABLE_LAP)" in lap

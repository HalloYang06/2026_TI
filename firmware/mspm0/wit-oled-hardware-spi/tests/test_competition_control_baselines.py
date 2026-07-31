from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]


def _function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_q2_verified_tracking_parameters_and_cadences_are_pinned() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    wheel_header = (
        PROJECT / "App" / "Control" / "wheel_control.h"
    ).read_text(encoding="utf-8")
    wheel_source = (
        PROJECT / "App" / "Control" / "wheel_control.c"
    ).read_text(encoding="utf-8")
    follower_source = (
        PROJECT / "App" / "Control" / "line_follower.c"
    ).read_text(encoding="utf-8")
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
        "config.base_speed = 63;",
        "config.initial_speed = 63;",
        "config.max_speed = 80;",
        "config.recovery_inner_speed = 24;",
        "config.recovery_outer_speed = 52;",
        "config.position_kp = 0.65f;",
        "config.steering_limit = 28;",
        "config.request_slew_step = 5;",
    ):
        assert statement in follower_source

    for statement in (
        "follower_profile = LINE_FOLLOWER_PROFILE_Q2_FAST_LAP;",
        "finish_line_min_run_ms = 18000U;",
        "run_timeout_ms = 0U;",
    ):
        assert statement in q2

    assert "#define WHEEL_CONTROL_PERIOD_MS 100U" in wheel_header
    assert "pid->Kp = 0.18f;" in wheel_source
    assert "pid->Ki = 0.005f;" in wheel_source
    assert wheel_source.count("initialize_pid(&control->") == 2
    assert "competition_runtime_wait_ms(10U);" in lap


def test_q4_keeps_verified_direct_start_without_unverified_ramp() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    follower_source = (
        PROJECT / "App" / "Control" / "line_follower.c"
    ).read_text(encoding="utf-8")
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
    assert "follower_profile = LINE_FOLLOWER_PROFILE_Q4_TIMED_RUN;" in q4
    assert "run_timeout_ms = 7800U;" in q4
    assert "config.base_speed = 50;" in follower_source
    assert "config.initial_speed = 50;" in follower_source
    assert "task2_start_speed" not in lap
    assert "task2_start_ramp_ms" not in lap
    assert "if (selected_task == CAR_TASK_STABLE_LAP)" in lap

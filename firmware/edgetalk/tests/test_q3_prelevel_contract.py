from pathlib import Path


EDGETALK = Path(__file__).resolve().parents[1]
APP = EDGETALK / "rtthread" / "hball_bench_app.c"
LOG_HEADER = EDGETALK / "include" / "hball_log_protocol.h"


def test_q3_uses_calibrated_level_and_feedback_initialized_slew() -> None:
    source = APP.read_text(encoding="utf-8")
    start = source[
        source.index("static int hball_q3_start_common(void)\n{") :
        source.index("static int hball_q3_start5", source.index(
            "static int hball_q3_start_common(void)\n{")
        )
    ]

    assert "g_hball_ball_level_rad = HBALL_BALL_COMMISSION_LEVEL_RAD;" in start
    assert "hball_fourbar_forward(" in start
    assert "g_hball_motor.parameters.mech_position_rad\n                - g_hball_ball_level_rad" in start
    assert "g_hball_ball_q3_pipe_command_rad = current_pipe_rad;" in start
    assert "g_hball_ball_output.motor_target_rad =\n        g_hball_motor.parameters.mech_position_rad;" in start
    assert "g_hball_ball_q3_leveling = RT_TRUE;" in start


def test_q3_prelevel_is_vision_independent_and_settles_before_pid() -> None:
    source = APP.read_text(encoding="utf-8")
    tick = source[
        source.index("static void hball_ball_commission_tick") :
        source.index("static void hball_motion_tick")
    ]
    leveling = tick[
        tick.index("if ((g_hball_ball_mode == 1U) && g_hball_ball_q3_leveling)") :
        tick.index("if ((g_hball_ball_mode == 1U) && vision_stale)")
    ]

    assert "HBALL_BALL_Q3_PIPE_RATE_LIMIT_RAD_S" in leveling
    assert "g_hball_ball_q3_leveling_first_target" in leveling
    assert "hball_fourbar_motor_offset(" in leveling
    assert "<= 0.002F" in leveling
    assert "<= 0.05F" in leveling
    assert ">= 100U" in leveling
    assert "g_hball_ball_position_integral = 0.0F;" in leveling
    assert "g_hball_ball_previous_error_m = 0.0F;" in leveling
    assert "g_hball_ball_start_ms = 0U;" in leveling
    assert "g_hball_ball_q3_leveling = RT_FALSE;" in leveling
    assert "hball_control_pipeline_step(" not in leveling


def test_q3_leveling_is_visible_in_status_and_binary_log() -> None:
    source = APP.read_text(encoding="utf-8")
    header = LOG_HEADER.read_text(encoding="utf-8")

    assert "HBALL_LOG_STATUS_Q3_LEVELING" in header
    assert "? HBALL_LOG_STATUS_Q3_LEVELING : 0U" in source
    assert "phase=%u leveling=%d passed=%d" in source

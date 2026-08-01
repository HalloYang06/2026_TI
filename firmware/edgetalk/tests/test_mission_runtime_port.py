from pathlib import Path


EDGETALK = Path(__file__).resolve().parents[1]


def test_m33_runtime_accepts_intent_and_publishes_standard_status() -> None:
    source = (EDGETALK / "rtthread" / "hball_bench_app.c").read_text(
        encoding="utf-8"
    )

    assert "hball_mission_decode_intent" in source
    assert "hball_mission_arbiter_accept_intent" in source
    assert "hball_mission_arbiter_update_ready" in source
    assert "hball_mission_action_tick" in source
    assert "hball_m33_q456_start_target" in source
    assert "hball_m33_q456_step" in source
    assert "hball_mission_arbiter_mark_running" in source
    assert "hball_mission_arbiter_mark_completed" in source
    assert "hball_mission_encode_status" in source
    assert "hball_send_mission_frame" in source
    assert "ACTUATOR_TX=0" in source


def test_edgetalk_build_includes_shared_mission_protocol() -> None:
    sconscript = (EDGETALK / "SConscript").read_text(encoding="utf-8")

    assert "HBALL_MISSION_PROTOCOL_ROOT" in sconscript
    assert "hball_mission_can.c" in sconscript
    assert "hball_mission_arbiter.c" in sconscript
    assert "hball_m33_q456.c" in sconscript


def test_manual_rs00_step_trace_is_bounded_and_read_only() -> None:
    source = (EDGETALK / "rtthread" / "hball_bench_app.c").read_text(
        encoding="utf-8"
    )

    assert "#define HBALL_RS00_STEP_TRACE_CAPACITY 120U" in source
    assert "#define HBALL_RS00_STEP_TRACE_DURATION_MS 1000U" in source
    assert "hball_motor_trace5" in source
    assert "g_hball_step_trace_count" in source
    assert "< HBALL_RS00_STEP_TRACE_CAPACITY" in source


def test_active_q456_restart_waits_for_target_and_retargets_cleanly() -> None:
    source = (EDGETALK / "rtthread" / "hball_bench_app.c").read_text(
        encoding="utf-8"
    )
    start_pending = source.index(
        "if (state == HBALL_MISSION_STATE_START_PENDING)"
    )
    fast_path_end = source.index(
        "if (g_hball_motion.state == HBALL_RS00_BENCH_PREPARED)",
        start_pending,
    )
    fast_path = source[start_pending:fast_path_end]
    mark_running = fast_path.index("hball_mission_arbiter_mark_running")

    for required_before_running in (
        "if (q456 && !q456_target_ready)",
        "g_hball_ball_target_m = q456_target_m;",
        "g_hball_ball_start_ms =",
        "g_hball_mission_arbiter.start_accept_time_ms;",
        "g_hball_ball_pipeline.controller.integral_error_m_s = 0.0F;",
        "g_hball_ball_position_integral = 0.0F;",
        "g_hball_ball_previous_error_m = 0.0F;",
        "g_hball_ball_settle_since_ms = 0U;",
        "g_hball_ball_settle_hold = RT_FALSE;",
    ):
        assert fast_path.index(required_before_running) < mark_running

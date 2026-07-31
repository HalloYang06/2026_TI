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

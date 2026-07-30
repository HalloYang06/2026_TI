from pathlib import Path


EDGETALK = Path(__file__).resolve().parents[1]


def test_m33_runtime_accepts_intent_and_publishes_standard_status() -> None:
    source = (EDGETALK / "rtthread" / "hball_bench_app.c").read_text(
        encoding="utf-8"
    )

    assert "hball_mission_decode_intent" in source
    assert "hball_mission_arbiter_accept_intent" in source
    assert "hball_mission_arbiter_update_ready" in source
    assert "hball_mission_encode_status" in source
    assert "hball_send_mission_frame" in source
    assert "ACTUATOR_TX=0" in source


def test_edgetalk_build_includes_shared_mission_protocol() -> None:
    sconscript = (EDGETALK / "SConscript").read_text(encoding="utf-8")

    assert "HBALL_MISSION_PROTOCOL_ROOT" in sconscript
    assert "hball_mission_can.c" in sconscript
    assert "hball_mission_arbiter.c" in sconscript

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]
M55_ADAPTER = (
    ROOT / "firmware" / "edgetalk" / "rtthread" / "hball_m55_shadow_app.c"
)
M55_SCONSCRIPT = ROOT / "firmware" / "edgetalk" / "SConscript.m55"
M55_UI = ROOT / "firmware" / "edgetalk" / "rtthread" / "hball_lvgl_ui.c"


def test_m55_runs_200hz_lqg_without_can_or_actuator_output():
    source = M55_ADAPTER.read_text(encoding="utf-8")
    exported = re.findall(r"MSH_CMD_EXPORT\((\w+),", source)

    assert exported == ["hball_m55_status"]
    assert "#define HBALL_M55_PERIOD_MS 5U" in source
    assert "rt_tick_from_millisecond(HBALL_M55_PERIOD_MS)" in source
    assert "rt_thread_delay_until" in source
    assert "rt_thread_mdelay(HBALL_M55_PERIOD_MS)" not in source
    assert "hball_m55_read_sensor_snapshot" in source
    assert "hball_control_pipeline_step" in source
    assert "safety_eligible" in source
    assert "ifx_can" not in source
    assert "rt_device_write" not in source
    assert "ACTUATOR_TX=0" in source


def test_m55_build_does_not_link_the_m33_can_monitor():
    sconscript = M55_SCONSCRIPT.read_text(encoding="utf-8")

    assert "hball_lqg.c" in sconscript
    assert "hball_control_pipeline.c" in sconscript
    assert "hball_m55_input_stub.c" in sconscript
    assert "hball_m55_shadow_app.c" in sconscript
    assert "hball_lvgl_ui.c" in sconscript
    assert "hball_can.c" not in sconscript
    assert "hball_bench_app.c" not in sconscript


def test_lvgl_page_is_h_problem_specific_and_has_no_rehab_arm_surface():
    source = M55_UI.read_text(encoding="utf-8")

    for label in [
        "BALL POSITION",
        "BALL VELOCITY",
        "IMU ACCEL",
        "YAW RATE",
        "MOTOR ANGLE",
        "LQG TARGET",
        "CAN RX",
        "VISION AGE",
    ]:
        assert label in source

    lowered = source.lower()
    for forbidden in ["rehab", "joint", "emg", "robot arm", "mechanical arm"]:
        assert forbidden not in lowered

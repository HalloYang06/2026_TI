from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]
ADAPTER = ROOT / "firmware" / "edgetalk" / "rtthread" / "hball_bench_app.c"
SCONSCRIPT = ROOT / "firmware" / "edgetalk" / "SConscript"


def test_rtthread_adapter_exposes_bounded_manual_motion_shell_commands():
    source = ADAPTER.read_text(encoding="utf-8")
    exported = re.findall(r"MSH_CMD_EXPORT\(\s*(\w+),", source)

    assert exported == [
        "hball_init",
        "hball_status",
        "hball_probe5",
        "hball_motor_prepare5",
        "hball_motor_arm5",
        "hball_motor_step5",
        "hball_motor_return5",
        "hball_motor_stop5",
        "hball_q3_start5",
        "hball_control_pid5",
        "hball_control_pid_gain5",
        "hball_control_pid_friction5",
        "hball_control_lqi5",
        "hball_hold_center5",
        "hball_hold_latch5",
        "hball_control_level5",
        "hball_q3_status5",
        "hball_motor_status5",
        "hball_motor_trace5",
    ]
    assert source.count("ifx_can_direct_send(") == 1
    assert source.count("ifx_can_direct_recv(") == 1
    assert "hball_motor_monitor_make_probe" in source
    assert "hball_runtime" not in source
    assert "hball_lqg" not in source
    assert "control_motor_" not in source
    assert "Cy_CANFD_UpdateAndTransmitMsgBuffer" not in source
    assert "hball_lqg_command" not in source
    assert "hball_rate_meter_accept(" in source
    assert "can_rate_x10=" in source
    assert "#define HBALL_RS00_READBACK_TX_ENABLED 0" in source
    assert "HBALL_RS00_READBACK_PERIOD_MS 20U" in source
    assert "hball_motor_monitor_make_parameter_read(" in source
    assert "#define HBALL_RS00_MOTION_TX_ENABLED 0" in source
    assert 'HBALL_RS00_CONFIRM_TOKEN "CONFIRM_NO_LOAD"' in source
    assert "hball_rs00_control_make_enable(" in source
    assert "hball_rs00_control_make_position_reference(" in source
    assert "hball_rs00_control_make_stop(" in source
    assert "set_zero" not in source.lower()
    assert "ACTUATOR_TX=0" in source


def test_bench_auto_probe_is_explicit_build_opt_in():
    source = ADAPTER.read_text(encoding="utf-8")
    sconscript = SCONSCRIPT.read_text(encoding="utf-8")

    assert "#define HBALL_BENCH_AUTO_PROBE5 0" in source
    assert "HBALL_BENCH_AUTO_PROBE5" in sconscript
    assert "os.environ.get('HBALL_BENCH_AUTO_PROBE5', '0')" in sconscript
    assert "hball_lqg.c" in sconscript
    assert "hball_runtime.c" not in sconscript
    assert "hball_rate_meter.c" in sconscript
    assert "HBALL_RS00_READBACK_TX_ENABLED=1" in sconscript
    assert "HBALL_RS00_MOTION_TX_ENABLED=1" in sconscript
    assert "hball_rs00_control.c" in sconscript


def test_manual_csp_session_uses_targeted_100_hz_each_position_velocity_readback():
    source = ADAPTER.read_text(encoding="utf-8")

    assert "#define HBALL_RS00_MOTION_READBACK_PERIOD_MS 5U" in source
    assert "HBALL_RS00_INDEX_MECH_POSITION" in source
    assert "HBALL_RS00_INDEX_MECH_VELOCITY" in source
    assert "hball_motion_poll_readback(now_ms);" in source
    assert "g_hball_motor.parameters.mech_position_rad" in source
    assert "g_hball_motor.parameters.mech_velocity_rad_s" in source
    assert "HBALL_RS00_BENCH_RETURNING" in source


def test_q3_verified_pid_and_sequence_baseline_is_frozen():
    source = ADAPTER.read_text(encoding="utf-8")

    for definition in (
        "#define HBALL_BALL_COMMISSION_LEVEL_RAD 1.7205F",
        "#define HBALL_BALL_COMMISSION_PIPE_LIMIT_RAD 0.052359878F",
        "#define HBALL_BALL_PID_KP 0.70F",
        "#define HBALL_BALL_PID_KI 0.15F",
        "#define HBALL_BALL_PID_KD 0.15F",
        "static float g_hball_ball_pid_static_boost_rad = 0.0F;",
    ):
        assert definition in source

    assert "if ((g_hball_ball_mode != 1U)" in source
    assert "g_hball_ball_target_m = 0.050F;" in source
    assert "g_hball_ball_target_m = -0.050F;" in source
    assert "#define HBALL_BALL_Q3_TARGET_RATE_MPS 0.20F" in source
    assert "#define HBALL_BALL_VISION_HOLD_MS 200U" in source
    assert "g_hball_ball_q3_zero_calibrated" in source
    assert "g_hball_ball_q3_vision_zero_m" in source
    assert "snapshot.ball_position_m -= g_hball_ball_q3_vision_zero_m" in source
    assert ">= 150U" in source
    assert ">= 300U" in source

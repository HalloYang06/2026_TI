from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCONSCRIPT = ROOT / "firmware" / "edgetalk" / "SConscript"
USB = ROOT / "firmware" / "edgetalk" / "rtthread" / "hball_usb_cdc.c"
CAN = ROOT / "firmware" / "edgetalk" / "rtthread" / "hball_bench_app.c"
INPUTS = ROOT / "firmware" / "edgetalk" / "rtthread" / "hball_m33_inputs.c"
GUARD = (
    ROOT
    / "firmware"
    / "edgetalk"
    / "rtthread"
    / "hball_m33_control_guard.c"
)


def test_integrated_mode_builds_usb_can_sensor_hub_and_manual_bench_path():
    sconscript = SCONSCRIPT.read_text(encoding="utf-8")

    assert "HBALL_INTEGRATED_SHADOW" in sconscript
    assert "hball_sensor_fusion.c" in sconscript
    assert "hball_m33_inputs.c" in sconscript
    assert "hball_usb_cdc.c" in sconscript
    assert "hball_bench_app.c" in sconscript
    assert "HBALL_INTEGRATED_SHADOW=1" in sconscript
    assert "HBALL_RS00_MOTION_TX_ENABLED=1" in sconscript
    assert "hball_rs00_control.c" in sconscript


def test_usb_and_can_publish_only_validated_measurements_to_m33_inputs():
    usb = USB.read_text(encoding="utf-8")
    can = CAN.read_text(encoding="utf-8")

    assert "hball_m33_inputs_publish_vision(" in usb
    assert "hball_msp_monitor_accept(" in can
    assert "hball_m33_inputs_publish_msp(" in can
    assert "hball_m33_inputs_publish_motor(" in can
    assert "hball_m33_inputs_publish_motor_parameters(" in can
    assert "HBALL_RS00_READBACK_TX_ENABLED=1" in SCONSCRIPT.read_text(
        encoding="utf-8"
    )
    assert "manual_motion_tx=%u" in can
    assert "ACTUATOR_TX=0" in can


def test_can_diagnostics_identify_the_integrated_shadow_image():
    can = CAN.read_text(encoding="utf-8")

    assert "#if HBALL_INTEGRATED_SHADOW" in can
    assert 'HBALL_BENCH_VERSION "0.5.0-m33-manual-small-step"' in can


def test_m33_input_hub_uses_mutex_and_200_hz_read_only_snapshots():
    source = INPUTS.read_text(encoding="utf-8")

    assert "rt_mutex_take(" in source
    assert "rt_mutex_release(" in source
    assert "#define HBALL_M33_SNAPSHOT_PERIOD_MS 5U" in source
    assert "hball_sensor_fusion_snapshot(" in source
    assert "ACTUATOR_TX=0" in source
    assert "ifx_can_direct_send" not in source


def test_m33_is_the_only_initializer_and_publishes_sensor_slot_at_200_hz():
    source = INPUTS.read_text(encoding="utf-8")

    assert '#include "hball_dualcore_platform.h"' in source
    assert "hball_ipc_region_reset(" in source
    assert "hball_ipc_sensor_publish(" in source
    assert "hball_ipc_platform_region()->sensor" in source
    assert "hball_ipc_platform_cache_ops()" in source
    assert "hball_ipc_control_publish(" not in source
    assert "HBALL_M33_SNAPSHOT_PERIOD_MS 5U" in source


def test_m33_observes_m55_shadow_at_1khz_without_any_actuator_path():
    source = GUARD.read_text(encoding="utf-8")
    sconscript = SCONSCRIPT.read_text(encoding="utf-8")

    assert "hball_control_guard.c" in sconscript
    assert "hball_m33_control_guard.c" in sconscript
    assert "#define HBALL_M33_GUARD_PERIOD_MS 1U" in source
    assert "rt_thread_delay_until(" in source
    assert "hball_ipc_control_read(" in source
    assert "hball_control_guard_observe(" in source
    assert "hball_control_guard_is_fresh(" in source
    assert "ACTUATOR_TX=0" in source
    for forbidden in [
        "ifx_can_direct_send",
        "Cy_CANFD_UpdateAndTransmitMsgBuffer",
        "rt_device_write",
    ]:
        assert forbidden not in source

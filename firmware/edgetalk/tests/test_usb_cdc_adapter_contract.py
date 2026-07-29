from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
ADAPTER = ROOT / "firmware" / "edgetalk" / "rtthread" / "hball_usb_cdc.c"
SCONSCRIPT = ROOT / "firmware" / "edgetalk" / "SConscript"
MAIN = ROOT / "firmware" / "edgetalk" / "rtthread" / "main.c"


def test_usb_cdc_uses_official_emusb_sequence_in_non_control_thread():
    source = ADAPTER.read_text(encoding="utf-8")

    assert 'rt_thread_create(' in source
    assert '"hball_usb"' in source
    assert "HBALL_USB_THREAD_PRIORITY 20U" in source
    assert '#include "USB.h"' in source
    assert '#include "USB_CDC.h"' in source
    device_init = source[source.index("static int hball_usb_device_init(void)") :]
    assert device_init.index("USBD_Init();") < device_init.index("hball_usb_add_cdc();")
    assert device_init.index("hball_usb_add_cdc();") < device_init.index("USBD_SetDeviceInfo(")
    assert device_init.index("USBD_SetDeviceInfo(") < device_init.index("USBD_Start();")
    assert "USBD_CDC_Add(&init_data)" in source
    assert "USB_STAT_CONFIGURED" in source
    assert "USB_STAT_SUSPENDED" in source
    assert "hball_usb_parse_ping" in source
    assert "hball_usb_format_pong" in source
    assert "hball_usb_format_ready" in source
    assert "cdc_acm_enter" not in source
    assert "ifx_can" not in source
    assert "MOTOR_ENABLE" not in source


def test_scons_only_builds_usb_adapter_when_emusb_is_enabled():
    sconscript = SCONSCRIPT.read_text(encoding="utf-8")

    assert "hball_usb_probe.c" in sconscript
    assert "hball_usb_cdc.c" in sconscript
    assert "emusb-device" in sconscript
    assert "BSP_USING_USB" in sconscript
    assert "CherryUSB" not in sconscript


def test_usb_only_build_is_explicit_and_excludes_can_sources():
    sconscript = SCONSCRIPT.read_text(encoding="utf-8")

    assert "os.environ.get('HBALL_USB_ONLY', '0')" in sconscript
    assert "if usb_only:" in sconscript
    assert "usb_only_src" in sconscript
    assert "hball_usb_probe.c" in sconscript
    assert "hball_usb_cdc.c" in sconscript

    usb_only_branch = sconscript.split("if usb_only:", 1)[1].split("else:", 1)[0]
    assert "hball_can.c" not in usb_only_branch
    assert "hball_bench_app.c" not in usb_only_branch
    assert "HBALL_USB_ONLY=1" in usb_only_branch


def test_m33_main_has_visible_p16_5_usb_only_heartbeat():
    source = MAIN.read_text(encoding="utf-8")

    assert "GET_PIN(16, 5)" in source
    assert "HBALL_HEARTBEAT_PERIOD_MS 500U" in source
    assert "rt_pin_mode" in source
    assert "rt_pin_write" in source
    assert "HBALL_USB_ONLY" in source

from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]
ADAPTER = ROOT / "firmware" / "edgetalk" / "rtthread" / "hball_bench_app.c"
SCONSCRIPT = ROOT / "firmware" / "edgetalk" / "SConscript"


def test_rtthread_adapter_exposes_only_read_only_shell_commands():
    source = ADAPTER.read_text(encoding="utf-8")
    exported = re.findall(r"MSH_CMD_EXPORT\((\w+),", source)

    assert exported == ["hball_init", "hball_status", "hball_probe5"]
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


def test_bench_auto_probe_is_explicit_build_opt_in():
    source = ADAPTER.read_text(encoding="utf-8")
    sconscript = SCONSCRIPT.read_text(encoding="utf-8")

    assert "#define HBALL_BENCH_AUTO_PROBE5 0" in source
    assert "HBALL_BENCH_AUTO_PROBE5" in sconscript
    assert "os.environ.get('HBALL_BENCH_AUTO_PROBE5', '0')" in sconscript
    assert "hball_lqg.c" not in sconscript
    assert "hball_runtime.c" not in sconscript
    assert "hball_rate_meter.c" in sconscript

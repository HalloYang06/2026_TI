from __future__ import annotations

from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[3]
CORE = ROOT / "firmware" / "edgetalk" / "src" / "hball_hold_controller.c"
ADAPTER = ROOT / "firmware" / "edgetalk" / "rtthread" / "hball_bench_app.c"


def test_decoupled_hold_controller_builds_and_passes_host_tests(tmp_path: Path):
    gcc = shutil.which("gcc")
    assert gcc is not None, "host GCC is required for the EdgeTalk C-port tests"

    executable = tmp_path / "hball_hold_controller_host_tests.exe"
    command = [
        gcc,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        "-I",
        str(ROOT / "firmware" / "edgetalk" / "include"),
        str(ROOT / "firmware" / "edgetalk" / "src" / "hball_hold_controller.c"),
        str(
            ROOT
            / "firmware"
            / "edgetalk"
            / "tests"
            / "hball_hold_controller_host_tests.c"
        ),
        "-lm",
        "-o",
        str(executable),
    ]
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    subprocess.run(
        [str(executable)], cwd=ROOT, check=True, capture_output=True, text=True
    )


def test_hold_core_has_no_runtime_transport_or_actuator_dependency():
    source = CORE.read_text(encoding="utf-8")

    for forbidden in (
        "rtthread",
        "finsh",
        "hball_can",
        "hball_mission",
        "hball_rs00",
        "hball_fourbar",
        "motor_target",
    ):
        assert forbidden not in source


def test_q3_is_frozen_while_q4_to_q6_share_the_hold_core():
    source = ADAPTER.read_text(encoding="utf-8")

    assert "#define HBALL_BALL_PID_KP 0.70F" in source
    assert "#define HBALL_BALL_PID_KI 0.15F" in source
    assert "#define HBALL_BALL_PID_KD 0.35F" in source
    assert "else if (g_hball_ball_mode != 1U)" in source
    assert "hball_hold_controller_step(" in source
    assert "HBALL_BALL_HOLD_KP" in source
    assert "HBALL_BALL_HOLD_INTEGRAL_LIMIT" in source
    first_start = source.index("static int hball_hold_start_common")
    hold_start = source.index(
        "static int hball_hold_start_common", first_start + 1
    )
    assert "HBALL_BALL_PID_INTEGRAL_LIMIT" not in source[
        hold_start : source.index("static int hball_hold_center5")
    ]
    assert 'hball_hold_start_common(\n                    2U, 0.0F, "center"' in source
    assert "3U, snapshot.ball_position_m, \"latched\"" in source

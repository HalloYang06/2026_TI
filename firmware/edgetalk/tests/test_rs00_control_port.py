from __future__ import annotations

from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[3]


def test_rs00_control_encoder_is_whitelisted_and_host_verifiable(tmp_path: Path):
    gcc = shutil.which("gcc")
    assert gcc is not None, "host GCC is required for the EdgeTalk C-port tests"

    executable = tmp_path / "hball_rs00_control_host_tests.exe"
    subprocess.run(
        [
            gcc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-I",
            str(ROOT / "firmware" / "edgetalk" / "include"),
            str(ROOT / "firmware" / "edgetalk" / "src" / "hball_can.c"),
            str(
                ROOT
                / "firmware"
                / "edgetalk"
                / "src"
                / "hball_rs00_control.c"
            ),
            str(
                ROOT
                / "firmware"
                / "edgetalk"
                / "tests"
                / "hball_rs00_control_host_tests.c"
            ),
            "-lm",
            "-o",
            str(executable),
        ],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        [str(executable)], cwd=ROOT, check=True, capture_output=True, text=True
    )


def test_rs00_control_api_exposes_no_zero_speed_or_torque_mode():
    header = (
        ROOT / "firmware" / "edgetalk" / "include" / "hball_rs00_control.h"
    ).read_text(encoding="utf-8")

    lowered = header.lower()
    for forbidden in ["set_zero", "torque_reference", "speed_reference", "iq_ref"]:
        assert forbidden not in lowered

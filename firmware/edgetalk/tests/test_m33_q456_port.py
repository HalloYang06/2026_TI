from __future__ import annotations

from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[3]


def test_q456_mission_runtime_host_contract(tmp_path: Path) -> None:
    gcc = shutil.which("gcc")
    assert gcc is not None, "host GCC is required for the EdgeTalk C-port tests"

    executable = tmp_path / "hball_m33_q456_host_tests.exe"
    command = [
        gcc,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        "-I",
        str(ROOT / "firmware" / "edgetalk" / "include"),
        "-I",
        str(ROOT / "shared" / "protocol"),
        str(
            ROOT
            / "firmware"
            / "edgetalk"
            / "rtthread"
            / "missions"
            / "hball_m33_q456.c"
        ),
        str(
            ROOT
            / "firmware"
            / "edgetalk"
            / "tests"
            / "hball_m33_q456_host_tests.c"
        ),
        "-lm",
        "-o",
        str(executable),
    ]
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    subprocess.run(
        [str(executable)], cwd=ROOT, check=True, capture_output=True, text=True
    )

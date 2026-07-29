from __future__ import annotations

from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[3]


def test_rate_meter_reports_stream_frequency_and_idle_decay(tmp_path: Path):
    gcc = shutil.which("gcc")
    assert gcc is not None, "host GCC is required for EdgeTalk C-port tests"

    executable = tmp_path / "hball_rate_meter_host_tests.exe"
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
            str(ROOT / "firmware" / "edgetalk" / "src" / "hball_rate_meter.c"),
            str(
                ROOT
                / "firmware"
                / "edgetalk"
                / "tests"
                / "hball_rate_meter_host_tests.c"
            ),
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

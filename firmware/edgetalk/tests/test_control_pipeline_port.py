from __future__ import annotations

from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[3]


def test_m55_control_pipeline_consumes_multirate_snapshots_safely(tmp_path: Path):
    gcc = shutil.which("gcc")
    assert gcc is not None, "host GCC is required for the EdgeTalk C-port tests"

    executable = tmp_path / "hball_control_pipeline_host_tests.exe"
    command = [
        gcc,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        "-I",
        str(ROOT / "firmware" / "edgetalk" / "include"),
        str(ROOT / "firmware" / "edgetalk" / "src" / "hball_deployment_controller.c"),
        str(ROOT / "firmware" / "edgetalk" / "src" / "hball_fourbar.c"),
        str(ROOT / "firmware" / "edgetalk" / "src" / "hball_imu_compensation.c"),
        str(ROOT / "firmware" / "edgetalk" / "src" / "hball_control_pipeline.c"),
        str(ROOT / "firmware" / "edgetalk" / "tests" / "hball_control_pipeline_host_tests.c"),
        "-lm",
        "-o",
        str(executable),
    ]
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    subprocess.run(
        [str(executable)], cwd=ROOT, check=True, capture_output=True, text=True
    )

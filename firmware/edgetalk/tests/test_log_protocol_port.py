import subprocess
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def test_log_protocol_host(tmp_path):
    gcc = shutil.which("gcc")
    assert gcc is not None, "host GCC is required for the EdgeTalk C-port tests"
    executable = tmp_path / "hball_log_protocol_tests"
    subprocess.run([
        gcc, "-std=c11", "-Wall", "-Wextra", "-Werror",
        "-I", str(ROOT / "firmware/edgetalk/include"),
        str(ROOT / "firmware/edgetalk/src/hball_log_protocol.c"),
        str(ROOT / "firmware/edgetalk/tests/hball_log_protocol_host_tests.c"),
        "-lm", "-o", str(executable),
    ], check=True)
    subprocess.run([str(executable)], check=True)

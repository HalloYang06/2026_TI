from pathlib import Path
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[3]
EDGETALK = ROOT / "firmware" / "edgetalk"
PROTOCOL = ROOT / "shared" / "protocol"


def test_m33_mission_arbiter_host_state_machine(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "hball_mission_arbiter_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{EDGETALK / 'include'}",
        f"-I{PROTOCOL}",
        str(PROTOCOL / "hball_mission_can.c"),
        str(EDGETALK / "src" / "hball_mission_arbiter.c"),
        str(EDGETALK / "tests" / "hball_mission_arbiter_host_tests.c"),
        "-o",
        str(executable),
    ]
    subprocess.run(command, cwd=ROOT, check=True)
    subprocess.run([str(executable)], cwd=ROOT, check=True)

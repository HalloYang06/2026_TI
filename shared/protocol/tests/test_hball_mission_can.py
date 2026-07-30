from pathlib import Path
import shutil
import subprocess


PROTOCOL_DIR = Path(__file__).resolve().parents[1]


def test_hball_mission_can_v1_host_contract(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "hball_mission_can_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{PROTOCOL_DIR}",
        str(PROTOCOL_DIR / "hball_mission_can.c"),
        str(Path(__file__).with_name("hball_mission_can_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROTOCOL_DIR)
    subprocess.run([str(executable)], check=True, cwd=PROTOCOL_DIR)

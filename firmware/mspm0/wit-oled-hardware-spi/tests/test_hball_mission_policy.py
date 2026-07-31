from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
MISSION_DIR = PROJECT / "App" / "Mission"
PROTOCOL_DIR = REPO / "shared" / "protocol"


def test_hball_mission_policy_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "hball_mission_policy_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{MISSION_DIR}",
        f"-I{PROTOCOL_DIR}",
        str(MISSION_DIR / "hball_mission_policy.c"),
        str(Path(__file__).with_name("hball_mission_policy_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)

from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
CAN_DIR = PROJECT / "Drivers" / "CAN"


def _host_compiler() -> str:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required for CAN recovery tests"
    return compiler


def test_hball_can_bus_off_recovery_contract(tmp_path: Path) -> None:
    executable = tmp_path / "hball_can_recovery_host_tests.exe"
    command = [
        _host_compiler(),
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{CAN_DIR}",
        str(CAN_DIR / "hball_can_recovery.c"),
        str(Path(__file__).with_name("hball_can_recovery_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)

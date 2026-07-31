from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
MISSION_DIR = PROJECT / "App" / "Mission"
RUNTIME_DIR = PROJECT / "App" / "Runtime"
PROTOCOL_DIR = REPO / "shared" / "protocol"


def test_runtime_services_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "hball_runtime_services_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{MISSION_DIR}",
        f"-I{RUNTIME_DIR}",
        f"-I{PROTOCOL_DIR}",
        str(MISSION_DIR / "hball_mission_policy.c"),
        str(RUNTIME_DIR / "hball_runtime_services.c"),
        str(Path(__file__).with_name("hball_runtime_services_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_runtime_service_gate_is_independent_of_hardware_drivers() -> None:
    header = (RUNTIME_DIR / "hball_runtime_services.h").read_text(
        encoding="utf-8"
    )
    source = (RUNTIME_DIR / "hball_runtime_services.c").read_text(
        encoding="utf-8"
    )
    combined = header + source

    assert "ti_msp_dl_config" not in combined
    assert "hball_can_port" not in combined
    assert "motor.h" not in combined
    assert "wit.h" not in combined

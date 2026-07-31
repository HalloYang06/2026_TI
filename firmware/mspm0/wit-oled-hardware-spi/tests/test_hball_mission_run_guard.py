from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
MISSION_DIR = PROJECT / "App" / "Mission"
CAN_DIR = PROJECT / "Drivers" / "CAN"
PROTOCOL_DIR = REPO / "shared" / "protocol"


def test_hball_mission_run_guard_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "hball_mission_run_guard_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{MISSION_DIR}",
        f"-I{CAN_DIR}",
        f"-I{PROTOCOL_DIR}",
        str(MISSION_DIR / "hball_mission_policy.c"),
        str(MISSION_DIR / "hball_mission_run_guard.c"),
        str(Path(__file__).with_name("hball_mission_run_guard_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_run_guard_is_pure_mission_logic() -> None:
    header = (MISSION_DIR / "hball_mission_run_guard.h").read_text(
        encoding="utf-8"
    )
    source = (MISSION_DIR / "hball_mission_run_guard.c").read_text(
        encoding="utf-8"
    )
    combined = header + source

    for forbidden in (
        "hball_can_port",
        "ti_msp_dl_config",
        "chassis_actuator",
        "motor.h",
        "LCD_",
        "DL_GPIO",
        "wit.h",
    ):
        assert forbidden not in combined


def test_keil_build_includes_mission_run_guard() -> None:
    project = (PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert "hball_mission_run_guard.c" in project
    assert "App\\Mission\\hball_mission_run_guard.c" in build
    assert "hball_mission_run_guard.c" in generator

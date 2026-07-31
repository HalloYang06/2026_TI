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


def test_policy_module_has_no_hardware_or_transport_dependency() -> None:
    header = (MISSION_DIR / "hball_mission_policy.h").read_text(
        encoding="utf-8"
    )
    source = (MISSION_DIR / "hball_mission_policy.c").read_text(
        encoding="utf-8"
    )
    combined = header + source

    assert "ti_msp_dl_config" not in combined
    assert "hball_can_port" not in combined
    assert "motor.h" not in combined
    assert "wit.h" not in combined


def test_keil_build_includes_mission_policy() -> None:
    project = (PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert "hball_mission_policy.c" in project
    assert "App\\Mission\\hball_mission_policy.c" in build
    assert "hball_mission_policy.c" in generator

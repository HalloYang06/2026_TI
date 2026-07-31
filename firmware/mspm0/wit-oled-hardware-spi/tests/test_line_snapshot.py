from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
CONTROL_DIR = PROJECT / "App" / "Control"


def test_line_snapshot_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "line_snapshot_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{CONTROL_DIR}",
        str(CONTROL_DIR / "line_snapshot.c"),
        str(Path(__file__).with_name("line_snapshot_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_line_snapshot_is_pure_control_data() -> None:
    header = (CONTROL_DIR / "line_snapshot.h").read_text(encoding="utf-8")
    source = (CONTROL_DIR / "line_snapshot.c").read_text(encoding="utf-8")
    combined = header + source

    forbidden_dependencies = (
        "ti_msp_dl_config",
        "chassis_actuator",
        "motor.h",
        "LCD_",
        "DL_GPIO",
        "hball_can",
        "wit.h",
    )
    for dependency in forbidden_dependencies:
        assert dependency not in combined


def test_keil_build_includes_line_snapshot() -> None:
    project = (PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert "line_snapshot.c" in project
    assert "App\\Control\\line_snapshot.c" in build
    assert "line_snapshot.c" in generator

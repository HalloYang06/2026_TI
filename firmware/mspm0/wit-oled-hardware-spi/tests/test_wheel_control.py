from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
CONTROL_DIR = PROJECT / "App" / "Control"
PID_DIR = PROJECT / "Drivers" / "PID"
FAKE_DIR = Path(__file__).resolve().parent / "fakes"


def test_wheel_control_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "wheel_control_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{FAKE_DIR}",
        f"-I{CONTROL_DIR}",
        f"-I{PID_DIR}",
        str(PID_DIR / "pid.c"),
        str(CONTROL_DIR / "wheel_control.c"),
        str(Path(__file__).with_name("wheel_control_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_wheel_control_has_no_hardware_transport_or_mission_dependency() -> None:
    header = (CONTROL_DIR / "wheel_control.h").read_text(encoding="utf-8")
    source = (CONTROL_DIR / "wheel_control.c").read_text(encoding="utf-8")
    combined = header + source

    for forbidden in (
        "ti_msp_dl_config",
        "chassis_actuator",
        "motor.h",
        "hball_can",
        "hball_mission",
        "line_sensor_port",
        "DL_GPIO",
    ):
        assert forbidden not in combined


def test_active_lap_runtime_delegates_wheel_pid_ownership() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    lap = main[
        main.index("static void lap_test_once(void)\n{") :
        main.index("void TIMER_0_INST_IRQHandler(void)")
    ]
    compact = "".join(lap.split())

    assert '#include "wheel_control.h"' in main
    assert "wheel_control_init(&wheel_control" in compact
    assert "wheel_control_reset_integrators(&wheel_control)" in compact
    assert "wheel_control_step(&wheel_control" in compact
    assert "PID_Update(&LEFT)" not in compact
    assert "PID_Update(&RIGHT)" not in compact
    assert "LEFT.ErrorInt" not in compact
    assert "RIGHT.ErrorInt" not in compact


def test_keil_build_includes_wheel_control() -> None:
    project = (PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert "wheel_control.c" in project
    assert "App\\Control\\wheel_control.c" in build
    assert "wheel_control.c" in generator

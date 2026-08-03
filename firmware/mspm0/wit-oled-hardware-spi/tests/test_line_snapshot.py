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


def test_active_lap_controller_consumes_line_snapshot_without_manual_decode() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    marker_source = (
        PROJECT / "App" / "Chassis" / "route_marker_detector.c"
    ).read_text(encoding="utf-8")
    start = main.index("static void lap_test_once(void)\n{")
    end = main.index("void TIMER_0_INST_IRQHandler(void)", start)
    lap = main[start:end]

    assert '#include "line_snapshot.h"' in main
    assert (
        lap.count(
            "line_snapshot_decode(line_sensor_port_read_raw(), tick_ms)"
        )
        >= 2
    )
    assert "static const int8_t weights[8]" not in lap
    assert "weighted_sum += weights[index];" not in lap
    assert "line_snapshot_has_adjacent(" in marker_source
    assert "route_marker_detector_step(" in lap
    assert "&line_sample," in lap

from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
CHASSIS_DIR = PROJECT / "App" / "Chassis"
CONTROL_DIR = PROJECT / "App" / "Control"


def test_route_marker_detector_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "route_marker_detector_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{CHASSIS_DIR}",
        f"-I{CONTROL_DIR}",
        str(CONTROL_DIR / "line_snapshot.c"),
        str(CHASSIS_DIR / "route_marker_detector.c"),
        str(Path(__file__).with_name("route_marker_detector_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_route_marker_detector_is_a_generic_hardware_free_fact_source() -> None:
    header = (CHASSIS_DIR / "route_marker_detector.h").read_text(
        encoding="utf-8"
    )
    source = (CHASSIS_DIR / "route_marker_detector.c").read_text(
        encoding="utf-8"
    )
    combined = header + source

    assert '#include "line_snapshot.h"' in header
    for forbidden in (
        "HBALL_MISSION_Q",
        "selected_task",
        "hball_can",
        "chassis_actuator",
        "wheel_control",
        "line_follower",
        "motor.h",
        "ti_msp_dl_config",
        "DL_GPIO",
        "LCD_",
        "tick_ms",
    ):
        assert forbidden not in combined


def test_route_marker_detector_is_in_both_keil_source_manifests() -> None:
    project = (PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert "route_marker_detector.c" in project
    assert "App\\Chassis\\route_marker_detector.c" in build
    assert "route_marker_detector.c" in generator


def test_active_lap_uses_route_marker_detector_facts() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    lap = main[
        main.index("static void lap_test_once(void)\n{") :
        main.index("void TIMER_0_INST_IRQHandler(void)")
    ]
    compact = "".join(lap.split())

    assert '#include "route_marker_detector.h"' in main
    assert "staticroute_marker_detector_troute_marker_detector;" in compact
    assert "route_marker_detector_config_tmarker_config;" in compact
    assert "route_marker_detector_output_tmarker_output;" in compact
    assert "marker_config.marker_active_threshold=3U;" in compact
    assert "marker_config.marker_adjacent_width=3U;" in compact
    assert "marker_config.marker_active_threshold=4U;" in compact
    assert "marker_config.marker_adjacent_width=4U;" in compact
    assert "marker_config.start_clear_confirm_ms=120U;" in compact
    assert "marker_config.marker_min_elapsed_ms=18000U;" in compact
    assert "marker_config.marker_min_elapsed_ms=23000U;" in compact
    assert "marker_config.marker_confirm_ms=20U;" in compact
    assert "route_marker_detector_init(" in compact
    assert "route_marker_detector_step(" in compact
    assert "marker_output.force_straight" in compact
    assert "marker_output.marker_confirmed" in compact

    for obsolete in (
        "finish_armed",
        "wide_finish_pattern",
        "finish_stop_confirmed",
        "start_line_clear_start_ms",
        "finish_candidate_start_ms",
        "line_snapshot_has_adjacent(&line_sample",
    ):
        assert obsolete not in lap

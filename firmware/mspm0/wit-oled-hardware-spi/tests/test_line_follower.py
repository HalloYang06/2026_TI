from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
CONTROL_DIR = PROJECT / "App" / "Control"


def test_line_follower_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "line_follower_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{CONTROL_DIR}",
        str(CONTROL_DIR / "line_snapshot.c"),
        str(CONTROL_DIR / "line_follower.c"),
        str(Path(__file__).with_name("line_follower_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_line_follower_has_no_hardware_transport_or_mission_dependency() -> None:
    header = (CONTROL_DIR / "line_follower.h").read_text(encoding="utf-8")
    source = (CONTROL_DIR / "line_follower.c").read_text(encoding="utf-8")
    combined = header + source

    for forbidden in (
        "ti_msp_dl_config",
        "line_sensor_port",
        "chassis_actuator",
        "wheel_control",
        "motor.h",
        "hball_can",
        "hball_mission",
        "DL_GPIO",
        "LCD_",
        "tick_ms",
    ):
        assert forbidden not in combined
    assert '#include "line_snapshot.h"' in header
    assert '#include "motion_intent.h"' in header


def test_active_lap_runtime_consumes_line_follower_intent() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    lap = main[
        main.index("static void lap_test_once(void)\n{") :
        main.index("void TIMER_0_INST_IRQHandler(void)")
    ]
    compact = "".join(lap.split())

    assert '#include "line_follower.h"' in main
    assert "line_follower_init(&line_follower" in compact
    assert "line_follower_step(&line_follower" in compact
    assert "wheel_intent=follower_output.intent;" in compact
    assert "line_follower_ack_motion_applied(&line_follower);" in compact
    assert "follower_profile=LINE_FOLLOWER_PROFILE_Q2_FAST_LAP;" in compact
    assert "follower_profile=LINE_FOLLOWER_PROFILE_Q4_TIMED_RUN;" in compact
    assert "follower_profile=LINE_FOLLOWER_PROFILE_STABLE_LAP;" in compact
    assert "follower_output.reset_wheel_integrators" in compact
    assert "follower_output.lost_timeout" in compact

    step_at = compact.index("line_follower_step(&line_follower")
    reset_at = compact.index("wheel_control_reset_integrators(&wheel_control)", step_at)
    wheel_at = compact.index("if(wheel_control_step(", reset_at)
    apply_at = compact.index("chassis_actuator_set_pwm(", wheel_at)
    ack_at = compact.index("line_follower_ack_motion_applied(&line_follower);", apply_at)
    assert step_at < reset_at < wheel_at < apply_at < ack_at

    assert "weighted_position_kp" not in lap
    assert "task3_filtered_error" not in lap
    assert "int16_trequested_speed_left" not in compact
    assert "int16_trequested_speed_right" not in compact


def test_keil_build_includes_line_follower() -> None:
    project = (PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert "line_follower.c" in project
    assert "App\\Control\\line_follower.c" in build
    assert "line_follower.c" in generator

from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
CONTROL_DIR = PROJECT / "App" / "Control"


def test_chassis_motion_profile_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"
    executable = tmp_path / "chassis_motion_profile_host_tests.exe"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{CONTROL_DIR}",
            str(CONTROL_DIR / "chassis_motion_profile.c"),
            str(Path(__file__).with_name("chassis_motion_profile_host_tests.c")),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=PROJECT,
    )
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_profile_has_no_hardware_or_mission_dependency() -> None:
    source = (CONTROL_DIR / "chassis_motion_profile.c").read_text(
        encoding="utf-8"
    )
    header = (CONTROL_DIR / "chassis_motion_profile.h").read_text(
        encoding="utf-8"
    )
    for forbidden in (
        "chassis_actuator",
        "motor.h",
        "hball_can",
        "hball_mission",
        "ti_msp_dl_config",
        "DL_GPIO",
    ):
        assert forbidden not in source + header


def test_q56_marker_stop_uses_nonblocking_profile_and_measured_settle() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    lap = main[
        main.index("static void lap_test_once(void)\n{") :
        main.index("void TIMER_0_INST_IRQHandler(void)")
    ]
    marker_stop = lap[
        lap.index(
            "if (local_marker_stop_enabled && "
            "marker_output.marker_confirmed_event)"
        ) :
        lap.index("(void)line_follower_step(", lap.index("q56_braking = true;"))
    ]
    compact = "".join(lap.split())

    assert "#define HBALL_Q56_SOFT_STOP_MS 900U" in main
    assert "#define HBALL_Q56_SOFT_START_MS 1200U" in main
    assert "#define HBALL_Q56_STOP_SPEED_THRESHOLD 2" in main
    assert "#define HBALL_Q56_STOP_SETTLE_MS 300U" in main
    assert (
        "#define HBALL_Q56_WHEEL_SAMPLE_STALE_MS "
        "(WHEEL_CONTROL_PERIOD_MS * 2U)" in main
    )
    assert "#define HBALL_Q56_STOP_FALLBACK_MS 600U" in main

    assert "q56_braking = true;" in marker_stop
    assert "chassis_motion_profile_start(" in marker_stop
    assert "q56_speed_scale," in marker_stop
    assert "HBALL_Q56_SOFT_STOP_MS" in marker_stop
    assert "while (" not in marker_stop
    assert "competition_runtime_wait_ms" not in marker_stop

    assert "elseif(selected_task==CAR_TASK_STABLE_LAP)" in compact
    assert "chassis_motion_profile_sample(" in compact
    assert compact.count("q56_speed_scale)") == 2
    assert "q56_braking&&!q4_speed_profile.active" in compact
    assert "wheel_sample_valid=true;" in compact
    assert "last_wheel_sample_ms=tick_ms;" in compact
    assert compact.count("HBALL_Q56_STOP_SPEED_THRESHOLD") == 4
    assert "q56_stop_settle_start_ms=tick_ms;" in compact
    assert ">=HBALL_Q56_STOP_SETTLE_MS" in compact
    assert "q56_stop_settle_active=false;" in compact
    profile_zero = compact.index("if(q56_stop_profile_complete)")
    assert compact.index(
        "chassis_actuator_set_pwm(0.0F,0.0F);", profile_zero
    ) < compact.index(
        "if(q56_stop_profile_complete&&!q56_stop_complete", profile_zero
    )
    assert ">HBALL_Q56_WHEEL_SAMPLE_STALE_MS" in compact
    assert ">=HBALL_Q56_STOP_FALLBACK_MS" in compact
    assert compact.index("if(q56_stop_complete)") < compact.rindex(
        "hball_can_mission_chassis_finish(finish_event_flags,tick_ms);"
    )

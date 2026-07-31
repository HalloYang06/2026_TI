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


def test_keil_build_includes_runtime_services() -> None:
    project = (PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert "hball_runtime_services.c" in project
    assert "..\\App\\Runtime" in project
    assert "App\\Runtime\\hball_runtime_services.c" in build
    assert "App\\Runtime" in build
    assert "hball_runtime_services.c" in generator
    assert "..\\App\\Runtime" in generator


def test_lap_runtime_applies_service_policy_before_motion() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    lap = main[
        main.index("static void lap_test_once(void)\n{") :
        main.index("void TIMER_0_INST_IRQHandler(void)")
    ]

    menu_services = lap.index("hball_runtime_services_enter_menu();")
    task_selection = lap.index("selected_task = select_car_task();")
    mission_policy = lap.index(
        "hball_mission_policy_get(selected_task, &mission_policy)"
    )
    apply_policy = lap.index(
        "hball_runtime_services_apply_policy(&mission_policy);"
    )
    first_motion = lap.index("motor_start_synchronized(")
    assert menu_services < task_selection < mission_policy < apply_policy < first_motion


def test_imu_interrupt_obeys_runtime_gate() -> None:
    interrupt = (
        PROJECT / "Drivers" / "MSPM0" / "interrupt.c"
    ).read_text(encoding="utf-8")
    wit = interrupt[
        interrupt.index("static void wit_process_dma_chunk(void)") :
        interrupt.index("void UART_WIT_INST_IRQHandler(void)")
    ]

    assert "process_imu = hball_runtime_services_imu_enabled();" in wit
    assert "if (process_imu)" in wit
    assert "WIT_ProcessBytes(" not in wit
    assert wit.count("WIT_QueueBytesFromISR(") == 2

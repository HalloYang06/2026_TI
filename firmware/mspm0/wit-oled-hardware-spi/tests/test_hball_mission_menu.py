from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
REPO = Path(__file__).resolve().parents[4]
CAN_DIR = PROJECT / "Drivers" / "CAN"
MISSION_DIR = PROJECT / "App" / "Mission"
PROTOCOL_DIR = REPO / "shared" / "protocol"


def test_hball_mission_menu_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "hball_mission_menu_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{CAN_DIR}",
        f"-I{MISSION_DIR}",
        f"-I{PROTOCOL_DIR}",
        str(PROTOCOL_DIR / "hball_mission_can.c"),
        str(CAN_DIR / "hball_mission_client.c"),
        str(MISSION_DIR / "hball_mission_menu.c"),
        str(Path(__file__).with_name("hball_mission_menu_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_msp_runtime_uses_distributed_menu_with_local_motion() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")

    assert "hball_can_mission_menu_handle" in main
    assert "hball_can_mission_get_snapshot" in main
    assert "HBALL_MISSION_LOCAL_MOTION_ENABLED 0U" in main
    assert "APP_MODE                 APP_MODE_CAN_TELEMETRY_TEST" in main
    assert "hball_can_mission_chassis_start" in main
    assert "hball_can_mission_chassis_finish" in main
    assert "Q2 FAST LAP" not in main
    assert "hball_mission_menu_view_equal" in main

    render = main.split("static void render_mission_menu(", 2)[2]
    render = render.split("static void format_hex16", 1)[0]
    assert "LCD_Fill(0, 0, LCD_W, LCD_H, BLACK)" not in render


def test_keil_build_includes_mission_menu_adapter() -> None:
    project = (
        PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx"
    ).read_text(encoding="utf-8")
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(encoding="utf-8")
    generator = (
        PROJECT / "tools" / "generate-keil-project.ps1"
    ).read_text(encoding="utf-8")

    assert "hball_mission_menu.c" in project
    assert "App\\Mission\\hball_mission_menu.c" in build
    assert "hball_mission_menu.c" in generator

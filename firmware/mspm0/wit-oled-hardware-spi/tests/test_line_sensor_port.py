from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
GRAY_DIR = PROJECT / "Drivers" / "GRAY"
FAKE_DIR = Path(__file__).resolve().parent / "fakes"


def test_line_sensor_port_preserves_all_gpio_bit_mappings(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "line_sensor_port_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{FAKE_DIR}",
        f"-I{GRAY_DIR}",
        str(GRAY_DIR / "line_sensor_port.c"),
        str(Path(__file__).with_name("line_sensor_port_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_line_sensor_port_is_the_only_track_gpio_reader() -> None:
    owner = GRAY_DIR / "line_sensor_port.c"
    violations = []

    for source_path in PROJECT.rglob("*.c"):
        if source_path == owner:
            continue
        source = source_path.read_text(encoding="utf-8", errors="ignore")
        if "DL_GPIO_readPins(track_PIN_" in source:
            violations.append(source_path.relative_to(PROJECT).as_posix())

    assert violations == []


def test_main_and_legacy_tracker_consume_the_shared_port() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    track = (GRAY_DIR / "track.c").read_text(encoding="utf-8")

    assert "static uint8_t read_track_raw(void)" not in main
    assert "line_sensor_port_read_raw()" in main
    assert "line_sensor_port_read_active_mask()" in track


def test_keil_build_includes_line_sensor_port() -> None:
    project = (PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert "line_sensor_port.c" in project
    assert "Drivers\\GRAY\\line_sensor_port.c" in build
    assert "line_sensor_port.c" in generator

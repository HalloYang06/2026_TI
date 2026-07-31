from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
WIT_DIR = PROJECT / "Drivers" / "WIT"


def test_wit_byte_queue_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "wit_byte_queue_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{WIT_DIR}",
        str(WIT_DIR / "wit_byte_queue.c"),
        str(Path(__file__).with_name("wit_byte_queue_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_keil_build_includes_wit_byte_queue() -> None:
    project = (PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert "wit_byte_queue.c" in project
    assert "Drivers\\WIT\\wit_byte_queue.c" in build
    assert "wit_byte_queue.c" in generator

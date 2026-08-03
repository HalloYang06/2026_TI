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


def test_wit_foreground_service_has_a_fixed_byte_budget() -> None:
    header = (WIT_DIR / "wit.h").read_text(encoding="utf-8")
    source = (WIT_DIR / "wit.c").read_text(encoding="utf-8")

    assert "#define WIT_FOREGROUND_BUDGET_PER_SERVICE 256U" in header
    assert "uint16_t WIT_QueueBytesFromISR(" in header
    assert "uint16_t WIT_Service(uint16_t max_bytes);" in header
    assert "wit_byte_queue_push(" in source
    assert "wit_byte_queue_pop(" in source
    assert "uint8_t buffer[WIT_DMA_TRANSFER_SIZE];" in source
    assert source.count("WIT_ProcessBytes(buffer, count);") == 1


def test_wit_queue_buffers_foreground_stalls() -> None:
    header = (WIT_DIR / "wit_byte_queue.h").read_text(encoding="utf-8")

    assert "#define WIT_BYTE_QUEUE_CAPACITY 1024U" in header

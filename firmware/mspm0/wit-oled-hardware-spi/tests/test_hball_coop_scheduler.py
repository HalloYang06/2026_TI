from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
RUNTIME_DIR = PROJECT / "App" / "Runtime"


def test_cooperative_scheduler_host_behavior(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"

    executable = tmp_path / "hball_coop_scheduler_host_tests.exe"
    command = [
        compiler,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{RUNTIME_DIR}",
        str(RUNTIME_DIR / "hball_coop_scheduler.c"),
        str(Path(__file__).with_name("hball_coop_scheduler_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)


def test_scheduler_core_has_no_hardware_or_rtos_dependency() -> None:
    header = (RUNTIME_DIR / "hball_coop_scheduler.h").read_text(
        encoding="utf-8"
    )
    source = (RUNTIME_DIR / "hball_coop_scheduler.c").read_text(
        encoding="utf-8"
    )
    combined = header + source

    for forbidden in (
        "ti_msp_dl_config",
        "FreeRTOS",
        "rtthread",
        "hball_can_port",
        "motor.h",
        "wit.h",
    ):
        assert forbidden not in combined

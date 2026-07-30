from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
WIT_DIR = PROJECT / "Drivers" / "WIT"


def test_jy901s_control_report_configuration(tmp_path: Path) -> None:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required"
    executable = tmp_path / "wit_jy901s_config_host_tests.exe"
    subprocess.run(
        [
            compiler,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            f"-I{WIT_DIR}",
            str(WIT_DIR / "wit_jy901s_config.c"),
            str(Path(__file__).with_name("wit_jy901s_config_host_tests.c")),
            "-o",
            str(executable),
        ],
        check=True,
        cwd=PROJECT,
    )
    subprocess.run([str(executable)], check=True, cwd=PROJECT)

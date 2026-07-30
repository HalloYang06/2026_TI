from pathlib import Path
import shutil
import subprocess


PROJECT = Path(__file__).resolve().parents[1]
WIT_DIR = PROJECT / "Drivers" / "WIT"


def _host_compiler() -> str:
    compiler = shutil.which("gcc") or shutil.which("clang")
    assert compiler is not None, "A host C compiler is required for WIT parser tests"
    return compiler


def test_wit_stream_parser_contract(tmp_path: Path) -> None:
    executable = tmp_path / "wit_parser_host_tests.exe"
    command = [
        _host_compiler(),
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        f"-I{WIT_DIR}",
        str(WIT_DIR / "wit_parser.c"),
        str(Path(__file__).with_name("wit_parser_host_tests.c")),
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True, cwd=PROJECT)
    subprocess.run([str(executable)], check=True, cwd=PROJECT)

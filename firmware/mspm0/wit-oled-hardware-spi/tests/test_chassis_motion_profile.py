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

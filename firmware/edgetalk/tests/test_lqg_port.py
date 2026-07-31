from __future__ import annotations

import math
from pathlib import Path
import shutil
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[3]
SIM_ROOT = ROOT / "experiments" / "h_ball_control_sim"
sys.path.insert(0, str(SIM_ROOT))

from hballsim.controllers import LQGController
from hballsim.model import PlantParameters


def test_edgetalk_lqg_port_matches_python_reference(tmp_path: Path):
    gcc = shutil.which("gcc")
    assert gcc is not None, "host GCC is required for the EdgeTalk C-port tests"

    executable = tmp_path / "hball_lqg_host_trace.exe"
    command = [
        gcc,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-pedantic",
        "-I",
        str(ROOT / "firmware" / "edgetalk" / "include"),
        str(ROOT / "firmware" / "edgetalk" / "src" / "hball_lqg.c"),
        str(ROOT / "firmware" / "edgetalk" / "tests" / "hball_lqg_host_trace.c"),
        "-lm",
        "-o",
        str(executable),
    ]
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
    completed = subprocess.run(
        [str(executable)], cwd=ROOT, check=True, capture_output=True, text=True
    )

    fields = completed.stdout.strip().split()
    assert fields[0] == "TRACE"
    c_position, c_velocity, c_bias, c_command = map(float, fields[1:5])
    c_accepted, c_rejected = map(int, fields[5:7])

    params = PlantParameters(
        beam_center_offset=0.080,
        beam_misalignment=math.radians(0.8),
        viscous_damping=0.20,
    )
    controller = LQGController(
        params=params,
        controller_dt=0.005,
        actuator_time_constant=0.032,
        camera_noise_std=0.0007,
        initial_position=0.004,
        beam_limit=math.radians(4.0),
        command_rate_limit=math.radians(80.0),
    )

    command_value = 0.0
    for index in range(20):
        beam_angle = 0.01 * math.sin(0.1 * index)
        body_pitch = 0.002 * math.cos(0.07 * index)
        longitudinal_accel = 0.10 + 0.002 * index
        lateral_accel = -0.04
        yaw_rate = 0.35
        controller.predict(
            dt=0.005,
            beam_angle=beam_angle,
            body_pitch=body_pitch,
            longitudinal_accel=longitudinal_accel,
            lateral_accel=lateral_accel,
            yaw_rate=yaw_rate,
        )
        if index % 3 == 0:
            controller.update_camera(0.004 + 0.0001 * index, 0.033)
        command_value = controller.command(
            target_position=0.0,
            actual_beam_angle=beam_angle,
            body_pitch=body_pitch,
            longitudinal_accel=longitudinal_accel,
            lateral_accel=lateral_accel,
            yaw_rate=yaw_rate,
        )

    assert math.isclose(c_position, controller.observer.state[0], abs_tol=2e-6)
    assert math.isclose(c_velocity, controller.observer.state[1], abs_tol=2e-5)
    assert math.isclose(c_bias, controller.observer.state[2], abs_tol=2e-5)
    assert math.isclose(c_command, command_value, abs_tol=2e-5)
    assert c_accepted == controller.observer.accepted_camera_updates
    assert c_rejected == controller.observer.rejected_camera_updates

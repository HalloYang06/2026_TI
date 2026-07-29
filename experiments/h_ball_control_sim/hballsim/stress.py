from __future__ import annotations

from dataclasses import replace
import math

import numpy as np

from .simulation import SimulationConfig


def impact_acceleration(
    time: float, events: tuple[tuple[float, float, float], ...]
) -> float:
    """Return the summed unmeasured ball acceleration from track impacts."""

    return sum(
        amplitude
        for start, duration, amplitude in events
        if start <= time < start + duration
    )


def generate_stress_case(
    *,
    level: float,
    trial: int,
    campaign_seed: int,
    duration: float = 25.0,
) -> SimulationConfig:
    """Build a deterministic, progressively harsher hardware stress case."""

    if level < 0.0:
        raise ValueError("level must be non-negative")
    seed = campaign_seed + trial * 10_007
    rng = np.random.default_rng(seed)

    dropout_count = int(math.ceil(level * 1.5))
    dropout_windows = []
    for _ in range(dropout_count):
        start = float(rng.uniform(2.0, max(2.1, duration - 1.0)))
        dropout_duration = level * float(rng.uniform(0.025, 0.060))
        dropout_windows.append((start, min(duration, start + dropout_duration)))

    impact_count = int(math.ceil(level * 2.0))
    impact_events = []
    for _ in range(impact_count):
        start = float(rng.uniform(1.0, max(1.1, duration - 0.5)))
        impact_duration = float(rng.uniform(0.020, 0.070))
        impact_amplitude = level * float(rng.uniform(0.07, 0.16))
        impact_amplitude *= -1.0 if rng.random() < 0.5 else 1.0
        impact_events.append((start, impact_duration, impact_amplitude))

    signed_accel = float(rng.uniform(-1.0, 1.0))
    signed_lateral = float(rng.uniform(-1.0, 1.0))
    signed_yaw = float(rng.uniform(-1.0, 1.0))
    signed_camera = float(rng.uniform(-1.0, 1.0))
    return replace(
        SimulationConfig(),
        duration=duration,
        random_seed=seed,
        initial_position=float(rng.uniform(-0.004, 0.004)),
        camera_period_jitter_std=level * 0.0008,
        camera_delay=0.033 + level * float(rng.uniform(0.006, 0.010)),
        camera_noise_std=0.0007 + level * float(rng.uniform(0.00025, 0.00040)),
        camera_outlier_probability=min(0.20, level * 0.004),
        camera_outlier_std=level * 0.004,
        camera_scale_error=level * signed_camera * 0.008,
        camera_bias=level * float(rng.uniform(-0.0004, 0.0004)),
        camera_dropout_windows=tuple(sorted(dropout_windows)),
        imu_delay=0.005 + level * 0.002,
        imu_accel_noise_std=0.010 + level * 0.008,
        imu_yaw_noise_std=0.004 + level * 0.003,
        pitch_noise_std=math.radians(0.03 + level * 0.025),
        imu_accel_bias=level * signed_accel * 0.020,
        imu_lateral_bias=level * signed_lateral * 0.020,
        imu_yaw_bias=level * signed_yaw * 0.006,
        imu_vibration_accel=level * 0.025,
        state_packet_dropout_probability=min(0.30, level * 0.012),
        beam_angle_noise_std=math.radians(0.015 + level * 0.015),
        beam_angle_quantization=math.radians(level * 0.010),
        actuator_time_constant=0.025 + level * 0.012,
        actuator_rate_limit=math.radians(160.0 / (1.0 + 0.25 * level)),
        actuator_deadband=math.radians(level * 0.040),
        actuator_command_delay=level * 0.004,
        actuator_gain=max(0.45, 1.0 - level * 0.08),
        impact_events=tuple(sorted(impact_events)),
        actual_beam_center_offset=0.085 + level * float(rng.uniform(-0.008, 0.008)),
        actual_beam_misalignment=math.radians(
            1.0 + level * float(rng.uniform(-0.7, 0.7))
        ),
        actual_viscous_damping=max(
            0.02, 0.28 + level * float(rng.uniform(-0.08, 0.10))
        ),
        actual_coulomb_accel=0.006 + level * float(rng.uniform(0.003, 0.008)),
        actual_static_friction_accel=level * float(rng.uniform(0.004, 0.012)),
        actual_turn_friction_accel_per_yaw=level
        * float(rng.uniform(0.004, 0.012)),
    )

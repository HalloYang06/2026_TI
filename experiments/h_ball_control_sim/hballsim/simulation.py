from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .controllers import LQGController
from .model import PlantParameters, ball_acceleration


@dataclass(frozen=True)
class SimulationConfig:
    duration: float = 25.0
    random_seed: int = 20260729
    plant_dt: float = 0.001
    controller_dt: float = 0.005
    camera_period: float = 1.0 / 100.0
    camera_period_jitter_std: float = 0.0
    camera_delay: float = 0.020
    camera_noise_std: float = 0.00035
    controller_camera_noise_std: float = 0.0007
    camera_outlier_probability: float = 0.0
    camera_outlier_std: float = 0.0
    camera_scale_error: float = 0.0
    camera_bias: float = 0.0
    imu_delay: float = 0.005
    imu_accel_noise_std: float = 0.010
    imu_yaw_noise_std: float = 0.004
    pitch_noise_std: float = math.radians(0.03)
    imu_accel_bias: float = 0.0
    imu_lateral_bias: float = 0.0
    imu_yaw_bias: float = 0.0
    imu_vibration_accel: float = 0.0
    imu_vibration_frequency: float = 23.0
    state_packet_dropout_probability: float = 0.0
    beam_angle_noise_std: float = math.radians(0.015)
    beam_angle_quantization: float = 0.0
    actuator_time_constant: float = 0.032
    controller_actuator_time_constant: float = 0.032
    actuator_rate_limit: float = math.radians(160.0)
    actuator_deadband: float = 0.0
    actuator_command_delay: float = 0.0
    actuator_gain: float = 1.0
    beam_command_rate_limit: float = math.radians(80.0)
    beam_limit: float = math.radians(4.0)
    initial_position: float = 0.004
    target_position: float = 0.0
    target_schedule: tuple[tuple[float, float], ...] = ()
    vehicle_motion_enabled: bool = True
    track_half_length: float = 0.120
    end_stop_restitution: float = 0.15
    camera_dropout_windows: tuple[tuple[float, float], ...] = ()
    impact_events: tuple[tuple[float, float, float], ...] = ()
    actual_beam_center_offset: float = 0.085
    actual_beam_misalignment: float = math.radians(1.0)
    actual_viscous_damping: float = 0.28
    actual_coulomb_accel: float = 0.006
    actual_static_friction_accel: float = 0.0
    actual_turn_friction_accel_per_yaw: float = 0.0


@dataclass(frozen=True)
class SimulationMetrics:
    peak_abs_error_m: float
    rms_error_m: float
    time_over_one_cm_s: float
    max_abs_beam_angle_rad: float
    final_error_m: float


@dataclass(frozen=True)
class SimulationResult:
    time: np.ndarray
    position: np.ndarray
    estimated_position: np.ndarray
    target: np.ndarray
    beam_angle: np.ndarray
    beam_command: np.ndarray
    longitudinal_accel: np.ndarray
    yaw_rate: np.ndarray
    body_pitch: np.ndarray
    metrics: SimulationMetrics
    lqr_gain: np.ndarray
    closed_loop_eigenvalues: np.ndarray


def _smooth_window(time: float, start: float, stop: float, ramp: float) -> float:
    def smoothstep(value: float) -> float:
        clipped = min(1.0, max(0.0, value))
        return clipped * clipped * (3.0 - 2.0 * clipped)

    return smoothstep((time - start) / ramp) * smoothstep((stop - time) / ramp)


def _scenario(time: float) -> tuple[float, float, float, float]:
    start_accel = 0.25 * _smooth_window(time, 0.35, 1.55, 0.20)
    first_curve_brake = -0.14 * _smooth_window(time, 4.15, 4.85, 0.18)
    first_curve_exit = 0.14 * _smooth_window(time, 9.05, 9.75, 0.18)
    second_curve_brake = -0.14 * _smooth_window(time, 13.35, 14.05, 0.18)
    second_curve_exit = 0.14 * _smooth_window(time, 18.25, 18.95, 0.18)
    final_brake = -0.25 * _smooth_window(time, 23.0, 24.2, 0.20)
    longitudinal_accel = (
        start_accel
        + first_curve_brake
        + first_curve_exit
        + second_curve_brake
        + second_curve_exit
        + final_brake
    )

    yaw_rate = 0.62 * (
        _smooth_window(time, 4.55, 9.35, 0.35)
        + _smooth_window(time, 13.75, 18.55, 0.35)
    )
    lateral_accel = -0.19 * (
        _smooth_window(time, 4.55, 9.35, 0.35)
        + _smooth_window(time, 13.75, 18.55, 0.35)
    )
    body_pitch = math.radians(0.18) * math.sin(2.0 * math.pi * 0.55 * time)
    return longitudinal_accel, lateral_accel, yaw_rate, body_pitch


def _scheduled_target(
    time: float,
    default_target: float,
    schedule: tuple[tuple[float, float], ...],
) -> float:
    target = default_target
    for start, scheduled_target in schedule:
        if time + 1e-12 < start:
            break
        target = scheduled_target
    return target


def run_scenario(controller_name: str, config: SimulationConfig) -> SimulationResult:
    if controller_name not in {"lqg", "lqg_no_feedforward"}:
        raise ValueError(f"Unsupported controller: {controller_name}")

    if not math.isclose(
        config.controller_dt / config.plant_dt,
        round(config.controller_dt / config.plant_dt),
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        raise ValueError("controller_dt must be an integer multiple of plant_dt")

    rng = np.random.default_rng(config.random_seed)
    actual_params = PlantParameters(
        beam_center_offset=config.actual_beam_center_offset,
        beam_misalignment=config.actual_beam_misalignment,
        viscous_damping=config.actual_viscous_damping,
        coulomb_accel=config.actual_coulomb_accel,
        static_friction_accel=config.actual_static_friction_accel,
        turn_friction_accel_per_yaw=config.actual_turn_friction_accel_per_yaw,
    )
    nominal_params = PlantParameters(
        beam_center_offset=0.080,
        beam_misalignment=math.radians(0.8),
        viscous_damping=0.20,
    )
    controller = LQGController(
        params=nominal_params,
        controller_dt=config.controller_dt,
        actuator_time_constant=config.controller_actuator_time_constant,
        camera_noise_std=config.controller_camera_noise_std,
        initial_position=config.initial_position,
        beam_limit=config.beam_limit,
        command_rate_limit=config.beam_command_rate_limit,
        feedforward_enabled=controller_name == "lqg",
    )

    step_count = int(round(config.duration / config.plant_dt)) + 1
    time_values = np.arange(step_count, dtype=float) * config.plant_dt
    position_values = np.zeros(step_count)
    estimated_values = np.zeros(step_count)
    target_values = np.zeros(step_count)
    beam_values = np.zeros(step_count)
    beam_command_values = np.zeros(step_count)
    accel_values = np.zeros(step_count)
    yaw_values = np.zeros(step_count)
    pitch_values = np.zeros(step_count)

    position = config.initial_position
    velocity = 0.0
    beam_angle = 0.0
    beam_command = 0.0
    applied_actuator_command = 0.0
    controller_stride = int(round(config.controller_dt / config.plant_dt))
    next_camera_capture = 0.0
    camera_queue: list[tuple[float, float, float]] = []
    actuator_command_queue: list[tuple[float, float]] = []
    held_imu_state = (0.0, 0.0, 0.0, 0.0)

    for index, time in enumerate(time_values):
        if config.vehicle_motion_enabled:
            longitudinal_accel, lateral_accel, yaw_rate, body_pitch = _scenario(time)
        else:
            longitudinal_accel, lateral_accel, yaw_rate, body_pitch = (
                0.0,
                0.0,
                0.0,
                0.0,
            )
        target_position = _scheduled_target(
            time, config.target_position, config.target_schedule
        )

        if time + 1e-12 >= next_camera_capture:
            capture_is_dropped = any(
                start <= next_camera_capture <= stop
                for start, stop in config.camera_dropout_windows
            )
            if not capture_is_dropped:
                measurement = (
                    position * (1.0 + config.camera_scale_error)
                    + config.camera_bias
                    + rng.normal(0.0, config.camera_noise_std)
                )
                if (
                    config.camera_outlier_probability > 0.0
                    and rng.random() < config.camera_outlier_probability
                ):
                    measurement += rng.normal(0.0, config.camera_outlier_std)
                camera_queue.append(
                    (
                        next_camera_capture + config.camera_delay,
                        next_camera_capture,
                        measurement,
                    )
                )
            next_period = config.camera_period
            if config.camera_period_jitter_std > 0.0:
                next_period += rng.normal(0.0, config.camera_period_jitter_std)
            next_camera_capture += max(0.25 * config.camera_period, next_period)

        if index % controller_stride == 0:
            (
                delayed_accel,
                delayed_lateral,
                delayed_yaw,
                delayed_pitch,
            ) = (
                _scenario(max(0.0, time - config.imu_delay))
                if config.vehicle_motion_enabled
                else (0.0, 0.0, 0.0, 0.0)
            )
            sensed_accel = delayed_accel + rng.normal(
                0.0, config.imu_accel_noise_std
            )
            vibration = config.imu_vibration_accel * math.sin(
                2.0 * math.pi * config.imu_vibration_frequency * time
            )
            sensed_accel += config.imu_accel_bias + vibration
            sensed_lateral = delayed_lateral + rng.normal(
                0.0, config.imu_accel_noise_std
            )
            sensed_lateral += config.imu_lateral_bias
            sensed_yaw = (
                delayed_yaw
                + config.imu_yaw_bias
                + rng.normal(0.0, config.imu_yaw_noise_std)
            )
            sensed_pitch = delayed_pitch + rng.normal(0.0, config.pitch_noise_std)
            packet_received = (
                index == 0
                or config.state_packet_dropout_probability <= 0.0
                or rng.random() >= config.state_packet_dropout_probability
            )
            if packet_received:
                held_imu_state = (
                    sensed_accel,
                    sensed_lateral,
                    sensed_yaw,
                    sensed_pitch,
                )
            sensed_accel, sensed_lateral, sensed_yaw, sensed_pitch = held_imu_state
            sensed_beam = beam_angle + rng.normal(
                0.0, config.beam_angle_noise_std
            )
            if config.beam_angle_quantization > 0.0:
                sensed_beam = (
                    round(sensed_beam / config.beam_angle_quantization)
                    * config.beam_angle_quantization
                )

            controller.predict(
                dt=config.controller_dt,
                beam_angle=sensed_beam,
                body_pitch=sensed_pitch,
                longitudinal_accel=sensed_accel,
                lateral_accel=sensed_lateral,
                yaw_rate=sensed_yaw,
            )
            while camera_queue and camera_queue[0][0] <= time + 1e-12:
                _, capture_time, measurement = camera_queue.pop(0)
                controller.update_camera(measurement, time - capture_time)

            beam_command = controller.command(
                target_position=target_position,
                actual_beam_angle=sensed_beam,
                body_pitch=sensed_pitch,
                longitudinal_accel=sensed_accel,
                lateral_accel=sensed_lateral,
                yaw_rate=sensed_yaw,
            )
            actuator_command_queue.append(
                (time + config.actuator_command_delay, beam_command)
            )

        while (
            actuator_command_queue
            and actuator_command_queue[0][0] <= time + 1e-12
        ):
            _, applied_actuator_command = actuator_command_queue.pop(0)

        actuator_target = float(
            np.clip(
                config.actuator_gain * applied_actuator_command,
                -config.beam_limit,
                config.beam_limit,
            )
        )
        actuator_error = actuator_target - beam_angle
        if abs(actuator_error) <= config.actuator_deadband:
            requested_rate = 0.0
        else:
            effective_error = actuator_error - math.copysign(
                config.actuator_deadband, actuator_error
            )
            requested_rate = effective_error / config.actuator_time_constant
        actual_rate = float(
            np.clip(
                requested_rate,
                -config.actuator_rate_limit,
                config.actuator_rate_limit,
            )
        )
        beam_angle += actual_rate * config.plant_dt
        beam_angle = float(np.clip(beam_angle, -config.beam_limit, config.beam_limit))

        acceleration = ball_acceleration(
            position=position,
            velocity=velocity,
            beam_angle=beam_angle,
            body_pitch=body_pitch,
            longitudinal_accel=longitudinal_accel,
            lateral_accel=lateral_accel,
            yaw_rate=yaw_rate,
            params=actual_params,
        )
        acceleration += sum(
            amplitude
            for start, duration, amplitude in config.impact_events
            if start <= time < start + duration
        )
        position += velocity * config.plant_dt + 0.5 * acceleration * config.plant_dt**2
        velocity += acceleration * config.plant_dt
        if abs(position) > config.track_half_length:
            position = math.copysign(config.track_half_length, position)
            if velocity * position > 0.0:
                velocity = -config.end_stop_restitution * velocity

        position_values[index] = position
        estimated_values[index] = controller.observer.state[0]
        target_values[index] = target_position
        beam_values[index] = beam_angle
        beam_command_values[index] = beam_command
        accel_values[index] = longitudinal_accel
        yaw_values[index] = yaw_rate
        pitch_values[index] = body_pitch

    error = position_values - target_values
    metrics = SimulationMetrics(
        peak_abs_error_m=float(np.max(np.abs(error))),
        rms_error_m=float(np.sqrt(np.mean(error**2))),
        time_over_one_cm_s=float(np.count_nonzero(np.abs(error) > 0.01) * config.plant_dt),
        max_abs_beam_angle_rad=float(np.max(np.abs(beam_values))),
        final_error_m=float(error[-1]),
    )
    closed_loop = controller.a_d - controller.b_d @ controller.gain.reshape(1, -1)
    return SimulationResult(
        time=time_values,
        position=position_values,
        estimated_position=estimated_values,
        target=target_values,
        beam_angle=beam_values,
        beam_command=beam_command_values,
        longitudinal_accel=accel_values,
        yaw_rate=yaw_values,
        body_pitch=pitch_values,
        metrics=metrics,
        lqr_gain=controller.gain,
        closed_loop_eigenvalues=np.linalg.eigvals(closed_loop),
    )

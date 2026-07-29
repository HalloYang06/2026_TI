from __future__ import annotations

import math
from collections import deque

import numpy as np

from .model import PlantParameters


class FrameVelocityEstimator:
    """Estimate current velocity from a short timestamped position window."""

    def __init__(self, window_size: int = 5) -> None:
        if window_size < 3:
            raise ValueError("window_size must be at least 3")
        self.samples: deque[tuple[float, float]] = deque(maxlen=window_size)

    def add(self, timestamp: float, position: float) -> None:
        if self.samples and timestamp <= self.samples[-1][0]:
            return
        self.samples.append((timestamp, position))

    @property
    def velocity(self) -> float:
        if len(self.samples) < 3:
            return 0.0
        timestamps = np.array([sample[0] for sample in self.samples])
        positions = np.array([sample[1] for sample in self.samples])
        relative_time = timestamps - timestamps[-1]
        design = np.column_stack(
            [np.ones_like(relative_time), relative_time, relative_time**2]
        )
        coefficients, *_ = np.linalg.lstsq(design, positions, rcond=None)
        return float(coefficients[1])

    @property
    def span(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        return self.samples[-1][0] - self.samples[0][0]


class BallStateObserver:
    """Small nonlinear Kalman observer for position, velocity and bias."""

    def __init__(
        self,
        *,
        params: PlantParameters,
        camera_noise_std: float,
        initial_position: float = 0.0,
        innovation_gate_sigma: float = 4.0,
    ) -> None:
        self.params = params
        self.state = np.array([initial_position, 0.0, 0.0], dtype=float)
        self.covariance = np.diag([4e-6, 4e-4, 2e-3])
        self.measurement_variance = camera_noise_std**2
        self.camera_noise_std = camera_noise_std
        self.innovation_gate_sigma = innovation_gate_sigma
        self.last_model_acceleration = 0.0
        self.current_time = 0.0
        self.frame_velocity = FrameVelocityEstimator(window_size=5)
        self.accepted_camera_updates = 0
        self.rejected_camera_updates = 0
        self.consecutive_camera_rejections = 0

    def predict(
        self,
        *,
        dt: float,
        beam_angle: float,
        body_pitch: float,
        longitudinal_accel: float,
        lateral_accel: float,
        yaw_rate: float,
    ) -> None:
        self.current_time += dt
        position, velocity, bias = self.state
        model_accel = (
            self.params.rolling_factor
            * (
                self.params.gravity * math.sin(beam_angle + body_pitch)
                - longitudinal_accel * math.cos(beam_angle + body_pitch)
                + yaw_rate**2 * (self.params.beam_center_offset + position)
                - lateral_accel * math.sin(self.params.beam_misalignment)
            )
            - self.params.viscous_damping * velocity
            + bias
        )
        self.state[0] = position + velocity * dt + 0.5 * model_accel * dt**2
        self.state[1] = velocity + model_accel * dt
        self.last_model_acceleration = model_accel

        accel_position_gradient = self.params.rolling_factor * yaw_rate**2
        accel_velocity_gradient = -self.params.viscous_damping
        transition = np.array(
            [
                [
                    1.0 + 0.5 * accel_position_gradient * dt**2,
                    dt + 0.5 * accel_velocity_gradient * dt**2,
                    0.5 * dt**2,
                ],
                [
                    accel_position_gradient * dt,
                    1.0 + accel_velocity_gradient * dt,
                    dt,
                ],
                [0.0, 0.0, 1.0],
            ]
        )
        process_noise = np.diag([2e-11, 2e-7, 2e-6])
        self.covariance = (
            transition @ self.covariance @ transition.T + process_noise
        )

    def update_delayed_position(self, measurement: float, age: float) -> bool:
        """Apply a camera measurement after predicting it to the current time."""

        predicted_current_measurement = (
            measurement
            + self.state[1] * age
            + 0.5 * self.last_model_acceleration * age**2
        )
        observation = np.array([[1.0, 0.0, 0.0]])
        innovation = predicted_current_measurement - (observation @ self.state).item()
        innovation_variance = (
            observation @ self.covariance @ observation.T
            + self.measurement_variance
        ).item()
        normalized_innovation_squared = innovation**2 / max(
            innovation_variance, 1e-12
        )
        if normalized_innovation_squared > self.innovation_gate_sigma**2:
            self.rejected_camera_updates += 1
            self.consecutive_camera_rejections += 1
            self.covariance[0, 0] += 4.0 * self.measurement_variance
            return False

        self.frame_velocity.add(self.current_time - age, measurement)
        gain = self.covariance @ observation.T / innovation_variance
        self.state = self.state + gain[:, 0] * innovation
        identity = np.eye(3)
        residual = identity - gain @ observation
        self.covariance = (
            residual @ self.covariance @ residual.T
            + gain * self.measurement_variance @ gain.T
        )

        if self.frame_velocity.span > 0.0:
            velocity_observation = np.array([[0.0, 1.0, 0.0]])
            velocity_variance = max(
                (2.0 * self.camera_noise_std / self.frame_velocity.span) ** 2,
                1e-5,
            )
            velocity_innovation = (
                self.frame_velocity.velocity
                - (velocity_observation @ self.state).item()
            )
            velocity_innovation_variance = (
                velocity_observation
                @ self.covariance
                @ velocity_observation.T
                + velocity_variance
            ).item()
            velocity_gain = (
                self.covariance
                @ velocity_observation.T
                / velocity_innovation_variance
            )
            self.state = self.state + velocity_gain[:, 0] * velocity_innovation
            velocity_residual = identity - velocity_gain @ velocity_observation
            self.covariance = (
                velocity_residual
                @ self.covariance
                @ velocity_residual.T
                + velocity_gain * velocity_variance @ velocity_gain.T
            )
        self.accepted_camera_updates += 1
        self.consecutive_camera_rejections = 0
        return True


class LQGController:
    """LQR state feedback with a model-based ball-state observer."""

    def __init__(
        self,
        *,
        params: PlantParameters,
        controller_dt: float,
        actuator_time_constant: float,
        camera_noise_std: float,
        initial_position: float,
        beam_limit: float,
        command_rate_limit: float,
        feedforward_enabled: bool = True,
    ) -> None:
        self.params = params
        self.beam_limit = beam_limit
        self.controller_dt = controller_dt
        self.command_rate_limit = command_rate_limit
        self.previous_command = 0.0
        self.feedforward_enabled = feedforward_enabled
        self.gain, self.a_d, self.b_d = discrete_lqr_gain(
            controller_dt=controller_dt,
            actuator_time_constant=actuator_time_constant,
            ball_input_gain=params.rolling_factor * params.gravity,
        )
        self.observer = BallStateObserver(
            params=params,
            camera_noise_std=camera_noise_std,
            initial_position=initial_position,
        )

    def predict(
        self,
        *,
        dt: float,
        beam_angle: float,
        body_pitch: float,
        longitudinal_accel: float,
        lateral_accel: float,
        yaw_rate: float,
    ) -> None:
        self.observer.predict(
            dt=dt,
            beam_angle=beam_angle,
            body_pitch=body_pitch,
            longitudinal_accel=longitudinal_accel,
            lateral_accel=lateral_accel,
            yaw_rate=yaw_rate,
        )

    def update_camera(self, measurement: float, age: float) -> bool:
        return self.observer.update_delayed_position(measurement, age)

    def command(
        self,
        *,
        target_position: float,
        actual_beam_angle: float,
        body_pitch: float,
        longitudinal_accel: float,
        lateral_accel: float,
        yaw_rate: float,
    ) -> float:
        position, velocity, bias = self.observer.state
        if self.feedforward_enabled:
            feedforward = compute_beam_feedforward(
                position=position,
                body_pitch=body_pitch,
                longitudinal_accel=longitudinal_accel,
                lateral_accel=lateral_accel,
                yaw_rate=yaw_rate,
                params=self.params,
            )
        else:
            feedforward = 0.0
        feedforward -= bias / (self.params.rolling_factor * self.params.gravity)
        feedback_state = np.array(
            [position - target_position, velocity, actual_beam_angle - feedforward]
        )
        requested_angle = float(
            np.clip(
                feedforward - float(self.gain @ feedback_state),
                -self.beam_limit,
                self.beam_limit,
            )
        )
        maximum_step = self.command_rate_limit * self.controller_dt
        command = float(
            np.clip(
                requested_angle,
                self.previous_command - maximum_step,
                self.previous_command + maximum_step,
            )
        )
        self.previous_command = command
        return command


def compute_beam_feedforward(
    *,
    position: float,
    body_pitch: float,
    longitudinal_accel: float,
    lateral_accel: float,
    yaw_rate: float,
    params: PlantParameters,
) -> float:
    """Solve the nonlinear steady-state beam angle for measured disturbances."""

    constant_term = (
        yaw_rate**2 * (params.beam_center_offset + position)
        - lateral_accel * math.sin(params.beam_misalignment)
    )
    amplitude = math.hypot(params.gravity, longitudinal_accel)
    phase = math.atan2(-longitudinal_accel, params.gravity)
    normalized = max(-1.0, min(1.0, -constant_term / amplitude))
    world_beam_angle = math.asin(normalized) - phase
    return world_beam_angle - body_pitch


def _matrix_exponential(matrix: np.ndarray) -> np.ndarray:
    """Compute exp(matrix) with a Taylor series for the small simulation matrices."""

    result = np.eye(matrix.shape[0])
    term = np.eye(matrix.shape[0])
    for index in range(1, 40):
        term = term @ matrix / index
        result = result + term
        if np.linalg.norm(term, ord=np.inf) < 1e-15:
            break
    return result


def discrete_lqr_gain(
    *,
    controller_dt: float,
    actuator_time_constant: float,
    ball_input_gain: float,
    q_weights: tuple[float, float, float] = (1200.0, 25.0, 2.0),
    r_weight: float = 5.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return a discrete LQR gain for [position, velocity, beam angle]."""

    a_c = np.array(
        [
            [0.0, 1.0, 0.0],
            [0.0, 0.0, ball_input_gain],
            [0.0, 0.0, -1.0 / actuator_time_constant],
        ]
    )
    b_c = np.array([[0.0], [0.0], [1.0 / actuator_time_constant]])

    augmented = np.zeros((4, 4))
    augmented[:3, :3] = a_c
    augmented[:3, 3:] = b_c
    discretized = _matrix_exponential(augmented * controller_dt)
    a_d = discretized[:3, :3]
    b_d = discretized[:3, 3:]

    q = np.diag(q_weights)
    r = np.array([[r_weight]])
    p = q.copy()
    for _ in range(20_000):
        denominator = r + b_d.T @ p @ b_d
        gain = np.linalg.solve(denominator, b_d.T @ p @ a_d)
        next_p = a_d.T @ p @ a_d - a_d.T @ p @ b_d @ gain + q
        if np.max(np.abs(next_p - p)) < 1e-12:
            p = next_p
            break
        p = next_p

    gain = np.linalg.solve(r + b_d.T @ p @ b_d, b_d.T @ p @ a_d)
    return gain.reshape(-1), a_d, b_d

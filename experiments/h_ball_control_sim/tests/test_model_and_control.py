import math
from pathlib import Path
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hballsim.controllers import (
    BallStateObserver,
    FrameVelocityEstimator,
    compute_beam_feedforward,
    discrete_lqr_gain,
)
from hballsim.model import PlantParameters, ball_acceleration
from hballsim.simulation import SimulationConfig, run_scenario


def test_level_stationary_beam_has_zero_ball_acceleration():
    params = PlantParameters()

    acceleration = ball_acceleration(
        position=0.0,
        velocity=0.0,
        beam_angle=0.0,
        body_pitch=0.0,
        longitudinal_accel=0.0,
        lateral_accel=0.0,
        yaw_rate=0.0,
        params=params,
    )

    assert acceleration == 0.0


def test_yaw_rate_adds_expected_longitudinal_centrifugal_term():
    params = PlantParameters(beam_center_offset=0.10, viscous_damping=0.0)

    acceleration = ball_acceleration(
        position=0.0,
        velocity=0.0,
        beam_angle=0.0,
        body_pitch=0.0,
        longitudinal_accel=0.0,
        lateral_accel=0.0,
        yaw_rate=0.6,
        params=params,
    )

    expected = (5.0 / 7.0) * 0.6**2 * 0.10
    assert math.isclose(acceleration, expected, rel_tol=1e-9, abs_tol=1e-12)


def test_physics_feedforward_cancels_acceleration_and_turning_disturbance():
    params = PlantParameters(beam_center_offset=0.08, viscous_damping=0.0)
    longitudinal_accel = 0.25
    yaw_rate = 0.55
    position = 0.03

    beam_angle = compute_beam_feedforward(
        position=position,
        body_pitch=0.0,
        longitudinal_accel=longitudinal_accel,
        lateral_accel=0.0,
        yaw_rate=yaw_rate,
        params=params,
    )
    acceleration = ball_acceleration(
        position=position,
        velocity=0.0,
        beam_angle=beam_angle,
        body_pitch=0.0,
        longitudinal_accel=longitudinal_accel,
        lateral_accel=0.0,
        yaw_rate=yaw_rate,
        params=params,
    )

    assert abs(acceleration) < 1e-4


def test_five_position_frames_estimate_ball_velocity():
    estimator = FrameVelocityEstimator(window_size=5)
    true_velocity = 0.08

    for index in range(7):
        timestamp = index / 60.0
        estimator.add(timestamp, 0.01 + true_velocity * timestamp)

    assert math.isclose(estimator.velocity, true_velocity, rel_tol=0.02)


def test_observer_rejects_an_extreme_camera_outlier_before_velocity_fusion():
    observer = BallStateObserver(
        params=PlantParameters(),
        camera_noise_std=0.0007,
        initial_position=0.0,
    )
    observer.predict(
        dt=0.005,
        beam_angle=0.0,
        body_pitch=0.0,
        longitudinal_accel=0.0,
        lateral_accel=0.0,
        yaw_rate=0.0,
    )
    state_before = observer.state.copy()

    accepted = observer.update_delayed_position(0.10, age=0.0)

    assert accepted is False
    assert np.allclose(observer.state, state_before)
    assert observer.rejected_camera_updates == 1
    assert len(observer.frame_velocity.samples) == 0


def test_observer_covariance_inflation_recovers_from_persistent_gate_rejections():
    observer = BallStateObserver(
        params=PlantParameters(),
        camera_noise_std=0.0012,
        initial_position=0.0,
    )
    accepted = False
    for _ in range(6):
        observer.predict(
            dt=1.0 / 60.0,
            beam_angle=0.0,
            body_pitch=0.0,
            longitudinal_accel=0.0,
            lateral_accel=0.0,
            yaw_rate=0.0,
        )
        accepted = observer.update_delayed_position(0.020, age=0.0)
        if accepted:
            break

    assert accepted is True
    assert observer.rejected_camera_updates > 0
    assert observer.rejected_camera_updates <= 5
    assert observer.accepted_camera_updates == 1


def test_static_friction_holds_small_drive_but_releases_on_larger_tilt():
    params = PlantParameters(
        viscous_damping=0.0,
        static_friction_accel=0.03,
        static_friction_speed=0.003,
    )
    held = ball_acceleration(
        position=0.0,
        velocity=0.0,
        beam_angle=math.radians(0.1),
        body_pitch=0.0,
        longitudinal_accel=0.0,
        lateral_accel=0.0,
        yaw_rate=0.0,
        params=params,
    )
    released = ball_acceleration(
        position=0.0,
        velocity=0.0,
        beam_angle=math.radians(0.5),
        body_pitch=0.0,
        longitudinal_accel=0.0,
        lateral_accel=0.0,
        yaw_rate=0.0,
        params=params,
    )

    assert held == 0.0
    assert released > 0.03


def test_turn_contact_friction_opposes_ball_velocity():
    params = PlantParameters(
        viscous_damping=0.0,
        turn_friction_accel_per_yaw=0.02,
    )
    straight = ball_acceleration(
        position=-params.beam_center_offset,
        velocity=0.05,
        beam_angle=0.0,
        body_pitch=0.0,
        longitudinal_accel=0.0,
        lateral_accel=0.0,
        yaw_rate=0.0,
        params=params,
    )
    turning = ball_acceleration(
        position=-params.beam_center_offset,
        velocity=0.05,
        beam_angle=0.0,
        body_pitch=0.0,
        longitudinal_accel=0.0,
        lateral_accel=0.0,
        yaw_rate=0.6,
        params=params,
    )

    assert turning < straight


def test_discrete_lqr_gain_is_finite_and_stabilizing():
    gain, a_d, b_d = discrete_lqr_gain(
        controller_dt=0.005,
        actuator_time_constant=0.025,
        ball_input_gain=(5.0 / 7.0) * 9.80665,
    )

    closed_loop_eigenvalues = np.linalg.eigvals(a_d - b_d @ gain.reshape(1, -1))

    assert np.all(np.isfinite(gain))
    assert np.max(np.abs(closed_loop_eigenvalues)) < 1.0


def test_lqg_controller_keeps_peak_error_below_one_centimeter():
    config = SimulationConfig(duration=25.0, random_seed=20260729)
    result = run_scenario("lqg", config)

    assert result.metrics.peak_abs_error_m < 0.01
    assert result.metrics.max_abs_beam_angle_rad <= math.radians(4.0) + 1e-9


def test_lqg_remains_stable_with_camera_dropout_and_slower_actuator():
    config = SimulationConfig(
        duration=25.0,
        random_seed=20260730,
        camera_delay=0.050,
        camera_noise_std=0.0012,
        actuator_time_constant=0.040,
        actuator_rate_limit=math.radians(100.0),
        camera_dropout_windows=((11.80, 11.88), (19.90, 19.98)),
    )

    result = run_scenario("lqg", config)

    assert result.metrics.peak_abs_error_m < 0.01
    assert result.metrics.rms_error_m < 0.003


def test_static_reference_sequence_reaches_plus_and_minus_five_centimeters():
    config = SimulationConfig(
        duration=5.0,
        random_seed=20260731,
        initial_position=0.0,
        vehicle_motion_enabled=False,
        target_schedule=((0.0, 0.050), (2.0, -0.050)),
    )

    result = run_scenario("lqg", config)

    before_reversal = (result.time >= 1.5) & (result.time < 2.0)
    assert np.min(np.abs(result.position[before_reversal] - 0.050)) < 0.010
    assert abs(result.position[-1] + 0.050) < 0.010


def test_moving_vehicle_holds_a_nonzero_commanded_ball_position():
    config = SimulationConfig(
        duration=25.0,
        random_seed=20260732,
        initial_position=0.050,
        target_position=0.050,
    )

    result = run_scenario("lqg", config)

    assert result.metrics.peak_abs_error_m < 0.010


def test_nonlinear_disturbance_feedforward_reduces_moving_vehicle_error():
    config = SimulationConfig(
        duration=25.0,
        random_seed=20260733,
        initial_position=0.0,
    )

    selected = run_scenario("lqg", config)
    ablation = run_scenario("lqg_no_feedforward", config)

    assert selected.metrics.rms_error_m < ablation.metrics.rms_error_m
    assert selected.metrics.peak_abs_error_m < ablation.metrics.peak_abs_error_m

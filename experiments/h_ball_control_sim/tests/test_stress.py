from pathlib import Path
import sys

from dataclasses import replace
import math
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hballsim.stress import generate_stress_case, impact_acceleration
from hballsim.simulation import SimulationConfig, run_scenario


def test_stress_generation_is_reproducible_for_the_same_seed_and_trial():
    first = generate_stress_case(level=2.0, trial=7, campaign_seed=20260729)
    second = generate_stress_case(level=2.0, trial=7, campaign_seed=20260729)

    assert first == second


def test_higher_stress_increases_delay_noise_deadband_and_packet_loss():
    low = generate_stress_case(level=1.0, trial=3, campaign_seed=20260729)
    high = generate_stress_case(level=3.0, trial=3, campaign_seed=20260729)

    assert high.camera_delay > low.camera_delay
    assert high.camera_noise_std > low.camera_noise_std
    assert high.actuator_deadband > low.actuator_deadband
    assert high.state_packet_dropout_probability > low.state_packet_dropout_probability
    assert len(high.impact_events) >= len(low.impact_events)


def test_track_impact_is_active_only_inside_its_time_window():
    events = ((2.0, 0.05, 0.40), (4.0, 0.10, -0.25))

    assert impact_acceleration(1.99, events) == 0.0
    assert impact_acceleration(2.02, events) == 0.40
    assert impact_acceleration(2.06, events) == 0.0
    assert impact_acceleration(4.05, events) == -0.25


def test_unmeasured_track_impact_changes_the_closed_loop_response():
    clean_config = SimulationConfig(
        duration=5.0,
        random_seed=42,
        initial_position=0.0,
    )
    impact_config = replace(
        clean_config,
        impact_events=((2.0, 0.10, 0.80),),
    )

    clean = run_scenario("lqg", clean_config)
    impacted = run_scenario("lqg", impact_config)

    assert not np.allclose(clean.position, impacted.position)
    assert impacted.metrics.peak_abs_error_m > clean.metrics.peak_abs_error_m


def test_camera_outliers_change_measurements_seen_by_the_observer():
    clean_config = SimulationConfig(duration=4.0, random_seed=7)
    outlier_config = replace(
        clean_config,
        camera_outlier_probability=1.0,
        camera_outlier_std=0.008,
    )

    clean = run_scenario("lqg", clean_config)
    outliers = run_scenario("lqg", outlier_config)

    assert not np.allclose(clean.estimated_position, outliers.estimated_position)


def test_camera_timing_scale_and_bias_change_the_estimated_trajectory():
    clean_config = SimulationConfig(duration=3.0, random_seed=11)
    stressed_config = replace(
        clean_config,
        camera_period_jitter_std=0.004,
        camera_scale_error=0.08,
        camera_bias=0.003,
    )

    clean = run_scenario("lqg", clean_config)
    stressed = run_scenario("lqg", stressed_config)

    assert not np.allclose(clean.estimated_position, stressed.estimated_position)


def test_imu_bias_vibration_and_state_packet_loss_change_control_action():
    clean_config = SimulationConfig(duration=2.0, random_seed=19)
    stressed_config = replace(
        clean_config,
        imu_accel_bias=0.08,
        imu_lateral_bias=-0.05,
        imu_yaw_bias=0.03,
        imu_vibration_accel=0.12,
        state_packet_dropout_probability=1.0,
    )

    clean = run_scenario("lqg", clean_config)
    stressed = run_scenario("lqg", stressed_config)

    assert not np.allclose(clean.beam_command, stressed.beam_command)


def test_actuator_command_delay_defers_beam_motion():
    clean_config = SimulationConfig(
        duration=0.08,
        random_seed=23,
        camera_noise_std=0.0,
        imu_accel_noise_std=0.0,
        imu_yaw_noise_std=0.0,
        pitch_noise_std=0.0,
        beam_angle_noise_std=0.0,
    )
    delayed_config = replace(clean_config, actuator_command_delay=0.040)

    clean = run_scenario("lqg", clean_config)
    delayed = run_scenario("lqg", delayed_config)

    assert np.max(np.abs(clean.beam_angle[:40])) > 0.0
    assert np.max(np.abs(delayed.beam_angle[:40])) == 0.0


def test_actuator_gain_deadband_and_encoder_quantization_change_response():
    clean_config = SimulationConfig(duration=2.0, random_seed=29)
    degraded_config = replace(
        clean_config,
        actuator_gain=0.55,
        actuator_deadband=math.radians(0.20),
        beam_angle_quantization=math.radians(0.10),
    )

    clean = run_scenario("lqg", clean_config)
    degraded = run_scenario("lqg", degraded_config)

    assert not np.allclose(clean.beam_angle, degraded.beam_angle)


def test_controller_keeps_nominal_lqr_gain_when_actual_actuator_slows():
    nominal_config = SimulationConfig(duration=1.0, random_seed=30)
    slowed_config = replace(nominal_config, actuator_time_constant=0.090)

    nominal = run_scenario("lqg", nominal_config)
    slowed = run_scenario("lqg", slowed_config)

    assert np.allclose(nominal.lqr_gain, slowed.lqr_gain)
    assert not np.allclose(nominal.beam_angle, slowed.beam_angle)


def test_conservative_camera_covariance_prevents_innovation_gate_lockout():
    regression_config = replace(
        generate_stress_case(
            level=2.0,
            trial=4,
            campaign_seed=20260729,
            duration=25.0,
        ),
        duration=2.0,
    )

    result = run_scenario("lqg", regression_config)

    assert result.metrics.peak_abs_error_m < 0.010


def test_configured_friction_and_geometry_change_the_plant_response():
    clean_config = SimulationConfig(duration=4.0, random_seed=31)
    changed_config = replace(
        clean_config,
        actual_beam_center_offset=0.12,
        actual_beam_misalignment=math.radians(5.0),
        actual_viscous_damping=0.65,
        actual_coulomb_accel=0.04,
        actual_static_friction_accel=0.03,
        actual_turn_friction_accel_per_yaw=0.04,
    )

    clean = run_scenario("lqg", clean_config)
    changed = run_scenario("lqg", changed_config)

    assert not np.allclose(clean.position, changed.position)


def test_physical_end_stops_bound_the_ball_position_during_extreme_failure():
    config = SimulationConfig(
        duration=2.0,
        random_seed=37,
        initial_position=0.0,
        impact_events=((0.25, 0.40, 12.0),),
    )

    result = run_scenario("lqg", config)

    assert np.max(np.abs(result.position)) <= config.track_half_length + 1e-12
    assert result.metrics.peak_abs_error_m > 0.01

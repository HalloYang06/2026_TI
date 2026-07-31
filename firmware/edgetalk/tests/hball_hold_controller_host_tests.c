#include "hball_hold_controller.h"

#include <assert.h>
#include <math.h>
#include <string.h>

static hball_hold_controller_input_t make_input(void)
{
    hball_hold_controller_input_t input;

    memset(&input, 0, sizeof(input));
    input.dt_s = 0.005F;
    input.feedback_enabled = true;
    input.feedforward_enabled = true;
    return input;
}

static void test_zero_state_holds_level(void)
{
    hball_hold_controller_config_t config;
    hball_hold_controller_t controller;
    hball_hold_controller_input_t input = make_input();
    hball_hold_controller_output_t output;

    hball_hold_controller_default_config(&config);
    hball_hold_controller_reset(&controller, 0.0F, 0.0F, 0.0F);
    assert(hball_hold_controller_step(&config, &controller, &input, &output));
    assert(fabsf(output.command_rad) < 1.0e-7F);
    assert(fabsf(output.position_error_m) < 1.0e-7F);
}

static void test_arbitrary_target_reuses_same_controller(void)
{
    hball_hold_controller_config_t config;
    hball_hold_controller_t controller;
    hball_hold_controller_input_t input = make_input();
    hball_hold_controller_output_t output;

    hball_hold_controller_default_config(&config);
    config.command_rate_limit_rad_s = 100.0F;
    input.target_position_m = 0.030F;
    input.estimated_position_m = 0.010F;
    input.feedforward_enabled = false;
    hball_hold_controller_reset(&controller, 0.0F, 0.0F, 0.0F);
    assert(hball_hold_controller_step(&config, &controller, &input, &output));
    assert(output.position_error_m > 0.0199F);
    assert(output.feedback_rad > 0.0139F);
    assert(output.command_rad > 0.0139F);
}

static void test_forward_acceleration_commands_positive_compensation(void)
{
    hball_hold_controller_config_t config;
    hball_hold_controller_t controller;
    hball_hold_controller_input_t input = make_input();
    hball_hold_controller_output_t output;

    hball_hold_controller_default_config(&config);
    config.accel_feedforward_gain = 1.0F;
    config.command_rate_limit_rad_s = 100.0F;
    input.longitudinal_accel_mps2 = 1.0F;
    hball_hold_controller_reset(&controller, 0.0F, 0.0F, 0.0F);
    assert(hball_hold_controller_step(&config, &controller, &input, &output));
    assert(output.filtered_accel_mps2 > 0.0F);
    assert(output.feedforward_rad > 0.0F);
    assert(output.command_rad > 0.0F);
    assert((output.flags & HBALL_HOLD_OUTPUT_FEEDFORWARD_ACTIVE) != 0U);
}

static void test_reset_bias_removes_static_imu_offsets(void)
{
    hball_hold_controller_config_t config;
    hball_hold_controller_t controller;
    hball_hold_controller_input_t input = make_input();
    hball_hold_controller_output_t output;

    hball_hold_controller_default_config(&config);
    config.accel_feedforward_gain = 1.0F;
    input.longitudinal_accel_mps2 = 0.35F;
    input.body_pitch_rad = 0.02F;
    hball_hold_controller_reset(&controller, 0.35F, 0.02F, 0.0F);
    assert(hball_hold_controller_step(&config, &controller, &input, &output));
    assert(fabsf(output.feedforward_rad) < 1.0e-7F);
}

static void test_raw_gravity_projection_is_cancelled_by_pitch(void)
{
    hball_hold_controller_config_t config;
    hball_hold_controller_t controller;
    hball_hold_controller_input_t input = make_input();
    hball_hold_controller_output_t output;
    const float pitch_rad = 0.05F;

    hball_hold_controller_default_config(&config);
    config.accel_feedforward_gain = 1.0F;
    config.accel_filter_tau_s = 0.0F;
    input.longitudinal_accel_mps2 = 9.80665F * tanf(pitch_rad);
    input.body_pitch_rad = pitch_rad;
    hball_hold_controller_reset(&controller, 0.0F, 0.0F, 0.0F);
    assert(hball_hold_controller_step(&config, &controller, &input, &output));
    assert(fabsf(output.feedforward_rad) < 1.0e-6F);
}

static void test_angle_rate_limit_and_anti_windup(void)
{
    hball_hold_controller_config_t config;
    hball_hold_controller_t controller;
    hball_hold_controller_input_t input = make_input();
    hball_hold_controller_output_t output;

    hball_hold_controller_default_config(&config);
    config.command_limit_rad = 0.010F;
    config.command_rate_limit_rad_s = 0.10F;
    input.target_position_m = 0.080F;
    input.feedforward_enabled = false;
    hball_hold_controller_reset(&controller, 0.0F, 0.0F, 0.0F);
    assert(hball_hold_controller_step(&config, &controller, &input, &output));
    assert((output.flags & HBALL_HOLD_OUTPUT_ANGLE_LIMITED) != 0U);
    assert((output.flags & HBALL_HOLD_OUTPUT_RATE_LIMITED) != 0U);
    assert(fabsf(output.command_rad - 0.0005F) < 1.0e-7F);
    assert(fabsf(controller.integral_error_m_s) < 1.0e-7F);
}

static void test_invalid_dt_is_rejected_without_state_change(void)
{
    hball_hold_controller_config_t config;
    hball_hold_controller_t controller;
    hball_hold_controller_input_t input = make_input();
    hball_hold_controller_output_t output;

    hball_hold_controller_default_config(&config);
    hball_hold_controller_reset(&controller, 0.0F, 0.0F, 0.0F);
    input.dt_s = 0.0F;
    assert(!hball_hold_controller_step(&config, &controller, &input, &output));
    assert(fabsf(controller.previous_command_rad) < 1.0e-7F);
}

int main(void)
{
    test_zero_state_holds_level();
    test_arbitrary_target_reuses_same_controller();
    test_forward_acceleration_commands_positive_compensation();
    test_reset_bias_removes_static_imu_offsets();
    test_raw_gravity_projection_is_cancelled_by_pitch();
    test_angle_rate_limit_and_anti_windup();
    test_invalid_dt_is_rejected_without_state_change();
    return 0;
}

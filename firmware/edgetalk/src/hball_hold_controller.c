#include "hball_hold_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define HBALL_HOLD_GRAVITY_MPS2 9.80665F
#define HBALL_HOLD_MAX_DT_S 0.050F

static float hball_hold_clampf(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static bool hball_hold_finite_config(
    const hball_hold_controller_config_t *config
)
{
    return isfinite(config->kp_rad_per_m)
        && isfinite(config->ki_rad_per_m_s)
        && isfinite(config->kd_rad_per_mps)
        && isfinite(config->integral_limit_m_s)
        && isfinite(config->accel_feedforward_gain)
        && isfinite(config->pitch_feedforward_gain)
        && isfinite(config->accel_filter_tau_s)
        && isfinite(config->command_limit_rad)
        && isfinite(config->command_rate_limit_rad_s);
}

void hball_hold_controller_default_config(
    hball_hold_controller_config_t *config
)
{
    if (config == NULL)
    {
        return;
    }
    config->kp_rad_per_m = 0.70F;
    config->ki_rad_per_m_s = 0.15F;
    config->kd_rad_per_mps = 0.35F;
    config->integral_limit_m_s = 0.050F;
    config->accel_feedforward_gain = 0.0F;
    config->pitch_feedforward_gain = 1.0F;
    config->accel_filter_tau_s = 0.030F;
    config->command_limit_rad = 0.052359878F;
    config->command_rate_limit_rad_s = 0.35F;
}

bool hball_hold_controller_config_valid(
    const hball_hold_controller_config_t *config
)
{
    return (config != NULL)
        && hball_hold_finite_config(config)
        && (config->kp_rad_per_m >= 0.0F)
        && (config->ki_rad_per_m_s >= 0.0F)
        && (config->kd_rad_per_mps >= 0.0F)
        && (config->integral_limit_m_s >= 0.0F)
        && (config->accel_feedforward_gain >= 0.0F)
        && (config->pitch_feedforward_gain >= 0.0F)
        && (config->accel_filter_tau_s >= 0.0F)
        && (config->command_limit_rad > 0.0F)
        && (config->command_rate_limit_rad_s > 0.0F);
}

void hball_hold_controller_reset(
    hball_hold_controller_t *controller,
    float accel_bias_mps2,
    float pitch_bias_rad,
    float initial_command_rad
)
{
    if (controller == NULL)
    {
        return;
    }
    memset(controller, 0, sizeof(*controller));
    if (!isfinite(accel_bias_mps2)
        || !isfinite(pitch_bias_rad)
        || !isfinite(initial_command_rad))
    {
        return;
    }
    controller->accel_bias_mps2 = accel_bias_mps2;
    controller->pitch_bias_rad = pitch_bias_rad;
    controller->previous_command_rad = initial_command_rad;
    controller->initialized = true;
}

static bool hball_hold_input_valid(
    const hball_hold_controller_input_t *input
)
{
    return (input != NULL)
        && isfinite(input->target_position_m)
        && isfinite(input->estimated_position_m)
        && isfinite(input->estimated_velocity_mps)
        && isfinite(input->longitudinal_accel_mps2)
        && isfinite(input->body_pitch_rad)
        && isfinite(input->dt_s)
        && (input->dt_s > 0.0F)
        && (input->dt_s <= HBALL_HOLD_MAX_DT_S);
}

static float hball_hold_feedback(
    const hball_hold_controller_config_t *config,
    float position_error_m,
    float integral_error_m_s,
    float estimated_velocity_mps
)
{
    return config->kp_rad_per_m * position_error_m
        + config->ki_rad_per_m_s * integral_error_m_s
        - config->kd_rad_per_mps * estimated_velocity_mps;
}

bool hball_hold_controller_step(
    const hball_hold_controller_config_t *config,
    hball_hold_controller_t *controller,
    const hball_hold_controller_input_t *input,
    hball_hold_controller_output_t *output
)
{
    float candidate_integral;
    float feedback_rad = 0.0F;
    float feedforward_rad = 0.0F;
    float requested_rad;
    float limited_rad;
    float maximum_change_rad;

    if (!hball_hold_controller_config_valid(config)
        || (controller == NULL)
        || !controller->initialized
        || !hball_hold_input_valid(input)
        || (output == NULL))
    {
        return false;
    }

    memset(output, 0, sizeof(*output));
    output->position_error_m =
        input->target_position_m - input->estimated_position_m;

    if (input->feedforward_enabled)
    {
        const float unbiased_accel = input->longitudinal_accel_mps2
            - controller->accel_bias_mps2;
        const float alpha = config->accel_filter_tau_s > 0.0F
            ? input->dt_s / (config->accel_filter_tau_s + input->dt_s)
            : 1.0F;

        controller->filtered_accel_mps2 += alpha
            * (unbiased_accel - controller->filtered_accel_mps2);
        feedforward_rad = config->accel_feedforward_gain
                * atan2f(
                    controller->filtered_accel_mps2,
                    HBALL_HOLD_GRAVITY_MPS2
                )
            - config->pitch_feedforward_gain
                * (input->body_pitch_rad - controller->pitch_bias_rad);
        output->flags |= HBALL_HOLD_OUTPUT_FEEDFORWARD_ACTIVE;
    }

    candidate_integral = controller->integral_error_m_s;
    if (input->feedback_enabled)
    {
        candidate_integral = hball_hold_clampf(
            candidate_integral + output->position_error_m * input->dt_s,
            -config->integral_limit_m_s,
            config->integral_limit_m_s
        );
        feedback_rad = hball_hold_feedback(
            config,
            output->position_error_m,
            candidate_integral,
            input->estimated_velocity_mps
        );
    }

    requested_rad = feedback_rad + feedforward_rad;
    limited_rad = hball_hold_clampf(
        requested_rad,
        -config->command_limit_rad,
        config->command_limit_rad
    );
    if (limited_rad != requested_rad)
    {
        const bool pushes_high = (requested_rad > 0.0F)
            && (output->position_error_m > 0.0F);
        const bool pushes_low = (requested_rad < 0.0F)
            && (output->position_error_m < 0.0F);

        output->flags |= HBALL_HOLD_OUTPUT_ANGLE_LIMITED;
        if (input->feedback_enabled && (pushes_high || pushes_low))
        {
            candidate_integral = controller->integral_error_m_s;
            feedback_rad = hball_hold_feedback(
                config,
                output->position_error_m,
                candidate_integral,
                input->estimated_velocity_mps
            );
            requested_rad = feedback_rad + feedforward_rad;
            limited_rad = hball_hold_clampf(
                requested_rad,
                -config->command_limit_rad,
                config->command_limit_rad
            );
        }
    }

    controller->integral_error_m_s = candidate_integral;
    maximum_change_rad = config->command_rate_limit_rad_s * input->dt_s;
    output->command_rad = controller->previous_command_rad
        + hball_hold_clampf(
            limited_rad - controller->previous_command_rad,
            -maximum_change_rad,
            maximum_change_rad
        );
    if (output->command_rad != limited_rad)
    {
        output->flags |= HBALL_HOLD_OUTPUT_RATE_LIMITED;
    }
    controller->previous_command_rad = output->command_rad;
    output->filtered_accel_mps2 = controller->filtered_accel_mps2;
    output->feedback_rad = feedback_rad;
    output->feedforward_rad = feedforward_rad;
    output->requested_rad = requested_rad;
    return true;
}

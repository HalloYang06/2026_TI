#ifndef HBALL_HOLD_CONTROLLER_H
#define HBALL_HOLD_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    HBALL_HOLD_OUTPUT_FEEDFORWARD_ACTIVE = UINT32_C(1) << 0,
    HBALL_HOLD_OUTPUT_ANGLE_LIMITED = UINT32_C(1) << 1,
    HBALL_HOLD_OUTPUT_RATE_LIMITED = UINT32_C(1) << 2,
};

typedef struct
{
    float kp_rad_per_m;
    float ki_rad_per_m_s;
    float kd_rad_per_mps;
    float integral_limit_m_s;
    float accel_feedforward_gain;
    float pitch_feedforward_gain;
    float accel_filter_tau_s;
    float command_limit_rad;
    float command_rate_limit_rad_s;
} hball_hold_controller_config_t;

typedef struct
{
    float integral_error_m_s;
    float accel_bias_mps2;
    float pitch_bias_rad;
    float filtered_accel_mps2;
    float previous_command_rad;
    bool initialized;
} hball_hold_controller_t;

typedef struct
{
    float target_position_m;
    float estimated_position_m;
    float estimated_velocity_mps;
    float longitudinal_accel_mps2;
    float body_pitch_rad;
    float dt_s;
    bool feedback_enabled;
    bool feedforward_enabled;
} hball_hold_controller_input_t;

typedef struct
{
    float position_error_m;
    float filtered_accel_mps2;
    float feedback_rad;
    float feedforward_rad;
    float requested_rad;
    float command_rad;
    uint32_t flags;
} hball_hold_controller_output_t;

void hball_hold_controller_default_config(
    hball_hold_controller_config_t *config
);

bool hball_hold_controller_config_valid(
    const hball_hold_controller_config_t *config
);

void hball_hold_controller_reset(
    hball_hold_controller_t *controller,
    float accel_bias_mps2,
    float pitch_bias_rad,
    float initial_command_rad
);

bool hball_hold_controller_step(
    const hball_hold_controller_config_t *config,
    hball_hold_controller_t *controller,
    const hball_hold_controller_input_t *input,
    hball_hold_controller_output_t *output
);

#ifdef __cplusplus
}
#endif

#endif

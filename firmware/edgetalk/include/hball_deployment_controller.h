#ifndef HBALL_DEPLOYMENT_CONTROLLER_H
#define HBALL_DEPLOYMENT_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_DEPLOYMENT_HISTORY_STEPS 32U

typedef struct
{
    float pipe_angle_rad;
    float body_pitch_rad;
    float longitudinal_accel_mps2;
    float lateral_accel_mps2;
    float yaw_rate_rad_s;
} hball_deployment_input_t;

typedef struct
{
    float state[3];
    float covariance[3][3];
    hball_deployment_input_t input;
    float dt_s;
    float time_s;
} hball_deployment_history_t;

typedef struct
{
    float state[3];
    float covariance[3][3];
    hball_deployment_history_t history[HBALL_DEPLOYMENT_HISTORY_STEPS];
    uint8_t history_count;
    float current_time_s;
    float integral_error_m_s;
    float previous_pipe_command_rad;
    hball_deployment_input_t conditioned_input;
    float longitudinal_accel_bias_mps2;
    float lateral_accel_bias_mps2;
    float body_pitch_bias_rad;
    bool imu_conditioner_initialized;
    uint32_t accepted_camera_updates;
    uint32_t rejected_camera_updates;
    uint32_t too_old_camera_updates;
    uint32_t edge_recovery_total;
} hball_deployment_controller_t;

void hball_deployment_controller_init(
    hball_deployment_controller_t *controller,
    float initial_position_m
);
void hball_deployment_controller_relock_position(
    hball_deployment_controller_t *controller,
    float measured_position_m
);
bool hball_deployment_controller_set_gains(
    float position_gain,
    float velocity_gain,
    float integral_gain
);
bool hball_deployment_controller_set_motion_compensation(
    float lateral_accel_coupling,
    float imu_filter_tau_s,
    float normal_angle_limit_rad
);
void hball_deployment_controller_predict(
    hball_deployment_controller_t *controller,
    float dt_s,
    const hball_deployment_input_t *input
);
bool hball_deployment_controller_update_delayed_position(
    hball_deployment_controller_t *controller,
    float measured_position_m,
    float age_s
);
float hball_deployment_controller_command(
    hball_deployment_controller_t *controller,
    const hball_deployment_input_t *input,
    float target_position_m,
    float dt_s,
    bool tracking_enabled
);

#ifdef __cplusplus
}
#endif

#endif

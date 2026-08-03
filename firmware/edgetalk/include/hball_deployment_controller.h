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
    /* Body +Y is forward; body +X is lateral on the current IMU mount. */
    float longitudinal_accel_mps2;
    float lateral_accel_mps2;
    /* Horizontal yaw of the pipe axis relative to body +Y. */
    float pipe_heading_offset_rad;
    float body_speed_mps;
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
    uint32_t accepted_camera_updates;
    uint32_t rejected_camera_updates;
    uint32_t too_old_camera_updates;
    uint32_t edge_recovery_total;
} hball_deployment_controller_t;

void hball_deployment_controller_init(
    hball_deployment_controller_t *controller,
    float initial_position_m
);
void hball_deployment_controller_predict(
    hball_deployment_controller_t *controller,
    float dt_s,
    const hball_deployment_input_t *input
);
/* Effective acceleration along the pipe, including the pipe-point yaw term. */
float hball_deployment_controller_accel_along_pipe(
    const hball_deployment_controller_t *controller,
    const hball_deployment_input_t *input
);
/* Signed lateral centripetal acceleration estimate: v * yaw_rate. */
float hball_deployment_estimate_lateral_centripetal_mps2(
    float body_speed_mps,
    float yaw_rate_rad_s
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

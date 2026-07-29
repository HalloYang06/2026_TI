#ifndef HBALL_LQG_H
#define HBALL_LQG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_VELOCITY_WINDOW_SIZE 5U

typedef struct
{
    float beam_angle_rad;
    float body_pitch_rad;
    float longitudinal_accel_mps2;
    float lateral_accel_mps2;
    float yaw_rate_rad_s;
} hball_lqg_input_t;

typedef struct
{
    float timestamp_s[HBALL_VELOCITY_WINDOW_SIZE];
    float position_m[HBALL_VELOCITY_WINDOW_SIZE];
    uint8_t count;
} hball_velocity_estimator_t;

typedef struct
{
    float state[3];
    float covariance[3][3];
    float last_model_accel_mps2;
    float current_time_s;
    float previous_command_rad;
    hball_velocity_estimator_t velocity_estimator;
    uint32_t accepted_camera_updates;
    uint32_t rejected_camera_updates;
    uint32_t consecutive_camera_rejections;
} hball_lqg_t;

void hball_lqg_init(hball_lqg_t *controller, float initial_position_m);
void hball_lqg_predict(
    hball_lqg_t *controller,
    float dt_s,
    const hball_lqg_input_t *input
);
bool hball_lqg_update_delayed_position(
    hball_lqg_t *controller,
    float measured_position_m,
    float age_s
);
float hball_lqg_command(
    hball_lqg_t *controller,
    const hball_lqg_input_t *input,
    float target_position_m
);

#ifdef __cplusplus
}
#endif

#endif

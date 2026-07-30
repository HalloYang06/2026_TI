#ifndef HBALL_M55_UI_H
#define HBALL_M55_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum
{
    HBALL_UI_VALID_IMU = 1U << 0,
    HBALL_UI_VALID_VISION = 1U << 1,
    HBALL_UI_VALID_MOTOR = 1U << 2,
    HBALL_UI_VALID_CAN = 1U << 3,
    HBALL_UI_VALID_MOTOR_PARAMETERS = 1U << 4,
};

typedef struct
{
    uint32_t valid_flags;
    uint32_t controller_steps;
    uint32_t deadline_misses;
    uint32_t can_rx_total;
    uint32_t vision_age_ms;
    uint8_t motor_mode_state;
    uint8_t motor_run_mode;
    uint8_t motor_fault_summary;
    uint8_t reserved;
    float ball_position_m;
    float ball_velocity_mps;
    float longitudinal_accel_mps2;
    float yaw_rate_rad_s;
    float motor_angle_rad;
    float motor_velocity_rad_s;
    float motor_temperature_c;
    float motor_filtered_iq_a;
    float motor_vbus_v;
    float lqg_target_rad;
} hball_m55_ui_snapshot_t;

void hball_m55_get_ui_snapshot(hball_m55_ui_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif

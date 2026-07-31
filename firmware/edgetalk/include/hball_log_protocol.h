#ifndef HBALL_LOG_PROTOCOL_H
#define HBALL_LOG_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HBALL_LOG_MAGIC UINT32_C(0x474C4248)
#define HBALL_LOG_VERSION 1U
#define HBALL_LOG_FRAME_SIZE 80U
#define HBALL_LOG_STATUS_SAFETY_QUALIFIED (UINT32_C(1) << 0)
#define HBALL_LOG_STATUS_Q3_ACTUAL (UINT32_C(1) << 16)
#define HBALL_LOG_STATUS_CONTROL_ACTIVE (UINT32_C(1) << 17)
#define HBALL_LOG_STATUS_Q3_PASSED (UINT32_C(1) << 18)

typedef struct
{
    uint32_t sequence;
    uint32_t produced_time_ms;
    uint32_t sensor_sequence;
    uint32_t controller_steps;
    uint32_t vision_sequence;
    uint32_t sensor_valid_flags;
    uint16_t control_mode;
    uint16_t guard_reason;
    uint32_t status_flags;
    float ball_position_m;
    float estimated_position_m;
    float estimated_velocity_mps;
    float estimated_disturbance_mps2;
    float pipe_target_rad;
    float motor_angle_rad;
    float motor_velocity_rad_s;
    float longitudinal_accel_mps2;
    float body_pitch_rad;
} hball_log_record_t;

uint32_t hball_log_crc32c(const uint8_t *data, size_t length);
bool hball_log_encode(
    const hball_log_record_t *record,
    uint8_t frame[HBALL_LOG_FRAME_SIZE]
);

#endif

#ifndef HBALL_LOG_PROTOCOL_H
#define HBALL_LOG_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HBALL_LOG_MAGIC UINT32_C(0x474C4248)
#define HBALL_LOG_VERSION 2U
#define HBALL_LOG_FRAME_SIZE 288U
#define HBALL_LOG_FLOAT_COUNT 39U
#define HBALL_LOG_STATUS_SAFETY_QUALIFIED (UINT32_C(1) << 0)
#define HBALL_LOG_STATUS_Q3_ACTUAL (UINT32_C(1) << 16)
#define HBALL_LOG_STATUS_CONTROL_ACTIVE (UINT32_C(1) << 17)
#define HBALL_LOG_STATUS_Q3_PASSED (UINT32_C(1) << 18)
#define HBALL_LOG_STATUS_ACTUAL_CONTROL (UINT32_C(1) << 19)

typedef struct
{
    uint32_t sequence;
    uint32_t produced_time_ms;
    uint32_t sensor_sequence;
    uint32_t controller_steps;
    uint32_t vision_sequence;
    uint32_t sensor_valid_flags;
    uint32_t status_flags;
    uint16_t control_mode;
    uint16_t guard_reason;
    uint64_t vision_capture_time_us;
    uint32_t vision_receive_time_ms;
    uint32_t vision_processing_time_us;
    uint16_t imu_epoch;
    uint16_t imu_sample_mask;
    uint32_t imu_source_time_ms;
    uint32_t accel_receive_time_ms;
    uint32_t gyro_receive_time_ms;
    uint32_t attitude_receive_time_ms;
    uint32_t imu_sync_receive_time_ms;
    uint16_t wheel_sequence;
    uint16_t accel_sequence;
    uint16_t gyro_sequence;
    uint16_t attitude_sequence;
    uint32_t wheel_receive_time_ms;
    uint32_t motor_receive_time_ms;
    uint16_t msp_status_flags;
    uint16_t vision_flags;
    uint8_t motor_fault_summary;
    uint8_t motor_mode_state;
    uint8_t motor_run_mode;
    uint8_t reserved;
    uint32_t control_output_flags;
    uint32_t vision_age_ms;
    uint32_t imu_age_ms;
    uint32_t wheel_age_ms;
    uint32_t motor_age_ms;
    uint32_t heartbeat_age_ms;
    float ball_position_m;
    float vision_confidence;
    float estimated_position_m;
    float estimated_velocity_mps;
    float target_position_m;
    float estimated_disturbance_mps2;
    float pipe_target_rad;
    float actual_pipe_angle_rad;
    float motor_target_rad;
    float motor_angle_rad;
    float motor_velocity_rad_s;
    float motor_torque_nm;
    float motor_temperature_c;
    float motor_filtered_iq_a;
    float motor_vbus_v;
    float accel_mps2[3];
    float gyro_rad_s[3];
    float attitude_rad[3];
    float wheel_left_mps;
    float wheel_right_mps;
    float body_speed_mps;
    float controller_position_error_m;
    float controller_integral_error_m_s;
    float controller_filtered_accel_mps2;
    float controller_feedback_rad;
    float controller_feedforward_rad;
    float controller_requested_rad;
    float controller_command_rad;
    float controller_p_rad;
    float controller_i_rad;
    float controller_d_rad;
    float controller_command_limit_rad;
    float controller_rate_limit_rad_s;
} hball_log_record_t;

uint32_t hball_log_crc32c(const uint8_t *data, size_t length);
bool hball_log_encode(
    const hball_log_record_t *record,
    uint8_t frame[HBALL_LOG_FRAME_SIZE]
);

#endif

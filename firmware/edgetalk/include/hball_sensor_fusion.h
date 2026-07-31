#ifndef HBALL_SENSOR_FUSION_H
#define HBALL_SENSOR_FUSION_H

#include "hball_can.h"
#include "hball_vision_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_SENSOR_VALID_VISION (UINT32_C(1) << 0)
#define HBALL_SENSOR_VALID_IMU (UINT32_C(1) << 1)
#define HBALL_SENSOR_VALID_WHEEL (UINT32_C(1) << 2)
#define HBALL_SENSOR_VALID_MOTOR (UINT32_C(1) << 3)
#define HBALL_SENSOR_VALID_HEARTBEAT (UINT32_C(1) << 4)
#define HBALL_SENSOR_ESTOP_ACTIVE (UINT32_C(1) << 5)
#define HBALL_SENSOR_VALID_MOTOR_PARAMETERS (UINT32_C(1) << 6)

#define HBALL_SENSOR_VISION_STALE_MS 100U
#define HBALL_SENSOR_IMU_STALE_MS 20U
#define HBALL_SENSOR_WHEEL_STALE_MS 30U
#define HBALL_SENSOR_MOTOR_STALE_MS 20U
#define HBALL_SENSOR_HEARTBEAT_STALE_MS 100U
#define HBALL_SENSOR_MOTOR_PARAMETER_STALE_MS 500U

typedef struct
{
    uint32_t sequence;
    uint32_t created_time_ms;
    uint32_t valid_flags;
    uint32_t vision_sequence;
    uint16_t accel_sequence;
    uint16_t gyro_sequence;
    uint16_t attitude_sequence;
    uint16_t wheel_sequence;
    uint16_t msp_status_flags;
    uint16_t vision_flags;
    uint16_t imu_epoch;
    uint16_t imu_sample_mask;
    uint8_t motor_fault_summary;
    uint8_t motor_mode_state;
    uint8_t motor_run_mode;
    uint8_t reserved;
    uint64_t vision_capture_time_us;
    uint32_t vision_processing_time_us;
    uint32_t vision_receive_time_ms;
    uint32_t imu_source_time_ms;
    uint32_t accel_receive_time_ms;
    uint32_t gyro_receive_time_ms;
    uint32_t attitude_receive_time_ms;
    uint32_t imu_sync_receive_time_ms;
    uint32_t wheel_receive_time_ms;
    uint32_t motor_receive_time_ms;
    uint32_t vision_receive_age_ms;
    uint32_t imu_age_ms;
    uint32_t wheel_age_ms;
    uint32_t motor_age_ms;
    uint32_t heartbeat_age_ms;
    float ball_position_m;
    float vision_confidence;
    float accel_mps2[3];
    float gyro_rad_s[3];
    float attitude_rad[3];
    float longitudinal_accel_mps2;
    float lateral_accel_mps2;
    float body_pitch_rad;
    float yaw_rate_rad_s;
    float body_speed_mps;
    float wheel_left_mps;
    float wheel_right_mps;
    float motor_angle_rad;
    float motor_velocity_rad_s;
    float motor_torque_nm;
    float motor_temperature_c;
    float motor_filtered_iq_a;
    float motor_vbus_v;
} hball_sensor_snapshot_t;

typedef struct
{
    hball_vision_measurement_t vision;
    hball_msp_monitor_t msp;
    hball_motor_feedback_t motor;
    hball_motor_parameters_t motor_parameters;
    uint32_t vision_receive_ms;
    uint32_t motor_receive_ms;
    uint32_t snapshot_sequence;
    bool vision_received;
    bool motor_received;
} hball_sensor_fusion_t;

void hball_sensor_fusion_init(hball_sensor_fusion_t *fusion);
void hball_sensor_fusion_set_vision(
    hball_sensor_fusion_t *fusion,
    const hball_vision_measurement_t *measurement,
    uint32_t receive_ms
);
void hball_sensor_fusion_set_msp(
    hball_sensor_fusion_t *fusion, const hball_msp_monitor_t *monitor
);
void hball_sensor_fusion_set_motor(
    hball_sensor_fusion_t *fusion,
    const hball_motor_feedback_t *feedback,
    uint32_t receive_ms
);
void hball_sensor_fusion_set_motor_parameters(
    hball_sensor_fusion_t *fusion,
    const hball_motor_parameters_t *parameters
);
void hball_sensor_fusion_snapshot(
    hball_sensor_fusion_t *fusion,
    uint32_t now_ms,
    hball_sensor_snapshot_t *snapshot
);

#ifdef __cplusplus
}
#endif

#endif

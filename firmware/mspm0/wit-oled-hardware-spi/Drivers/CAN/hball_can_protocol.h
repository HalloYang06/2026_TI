#ifndef HBALL_CAN_PROTOCOL_H
#define HBALL_CAN_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_CAN_CLASSIC_BITRATE UINT32_C(1000000)

#define HBALL_MSP_CAN_ID_HEARTBEAT UINT32_C(0x080)
#define HBALL_MSP_CAN_ID_ACCEL UINT32_C(0x100)
#define HBALL_MSP_CAN_ID_GYRO UINT32_C(0x101)
#define HBALL_MSP_CAN_ID_WHEEL UINT32_C(0x102)
#define HBALL_MSP_CAN_ID_ATTITUDE UINT32_C(0x103)
#define HBALL_MSP_CAN_ID_IMU_TIME UINT32_C(0x104)

#define HBALL_MSP_IMU_SAMPLE_ACCEL (UINT16_C(1) << 0)
#define HBALL_MSP_IMU_SAMPLE_GYRO (UINT16_C(1) << 1)
#define HBALL_MSP_IMU_SAMPLE_ATTITUDE (UINT16_C(1) << 2)
#define HBALL_MSP_IMU_SAMPLE_COMPLETE (UINT16_C(1) << 3)

#define HBALL_MSP_STATUS_ESTOP_ACTIVE (UINT16_C(1) << 0)
#define HBALL_MSP_STATUS_IMU_VALID (UINT16_C(1) << 1)
#define HBALL_MSP_STATUS_CHASSIS_READY (UINT16_C(1) << 2)
#define HBALL_MSP_STATUS_LOCAL_CONTROL_ACTIVE (UINT16_C(1) << 3)

typedef enum
{
    HBALL_CAN_STREAM_ACCEL = 0,
    HBALL_CAN_STREAM_GYRO,
    HBALL_CAN_STREAM_WHEEL,
    HBALL_CAN_STREAM_ATTITUDE,
    HBALL_CAN_STREAM_IMU_TIME,
    HBALL_CAN_STREAM_HEARTBEAT,
    HBALL_CAN_STREAM_COUNT,
} hball_can_stream_t;

typedef struct
{
    uint32_t id;
    uint8_t is_extended;
    uint8_t is_remote;
    uint8_t dlc;
    uint8_t fdf;
    uint8_t brs;
    uint8_t data[8];
} hball_can_frame_t;

typedef struct
{
    uint16_t status_flags;
    uint32_t uptime_ms;
    uint16_t accel_source_sequence;
    uint16_t gyro_source_sequence;
    uint16_t attitude_source_sequence;
    uint16_t imu_epoch;
    uint16_t imu_sample_mask;
    uint32_t imu_source_time_ms;
    int16_t accel_milli_mps2[3];
    int16_t gyro_milli_rad_s[3];
    int16_t wheel_milli_mps[2];
    int16_t body_milli_mps;
    int16_t attitude_milli_rad[3];
} hball_can_inputs_t;

bool hball_can_stream_due(uint32_t now_ms, hball_can_stream_t *stream);
bool hball_can_encode_frame(
    hball_can_stream_t stream,
    uint16_t sequence,
    const hball_can_inputs_t *inputs,
    hball_can_frame_t *frame
);

int16_t hball_can_mg_to_milli_mps2(int16_t acceleration_mg);
int16_t hball_can_dps_to_milli_rad_s(int16_t angular_velocity_dps);
int16_t hball_can_deg_to_milli_rad(float angle_deg);
int16_t hball_can_counts_per_20ms_to_milli_mps(
    int32_t counts,
    uint32_t wheel_circumference_mm,
    uint32_t counts_per_revolution
);

#ifdef __cplusplus
}
#endif

#endif

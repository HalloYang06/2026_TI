#ifndef HBALL_CAN_H
#define HBALL_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_CAN_CLASSIC_BITRATE 1000000UL
#define HBALL_RS00_MASTER_ID 0xFDU
#define HBALL_RS00_MOTOR_ID 0x05U
#define HBALL_RS00_GET_ID_REPLY 0xFEU

#define HBALL_RS00_TYPE_GET_ID 0x00U
#define HBALL_RS00_TYPE_FEEDBACK 0x02U
#define HBALL_RS00_TYPE_ACTIVE_REPORT 0x18U

#define HBALL_MSP_CAN_ID_HEARTBEAT 0x080U
#define HBALL_MSP_CAN_ID_ACCEL 0x100U
#define HBALL_MSP_CAN_ID_GYRO 0x101U
#define HBALL_MSP_CAN_ID_WHEEL 0x102U
#define HBALL_MSP_CAN_ID_ATTITUDE 0x103U

#define HBALL_MSP_STATUS_ESTOP_ACTIVE (UINT16_C(1) << 0)
#define HBALL_MSP_STATUS_IMU_VALID (UINT16_C(1) << 1)
#define HBALL_MSP_STATUS_CHASSIS_READY (UINT16_C(1) << 2)
#define HBALL_MSP_STATUS_LOCAL_CONTROL_ACTIVE (UINT16_C(1) << 3)

#define HBALL_RS00_POSITION_MIN_RAD (-12.57F)
#define HBALL_RS00_POSITION_MAX_RAD (12.57F)
#define HBALL_RS00_VELOCITY_MIN_RAD_S (-33.0F)
#define HBALL_RS00_VELOCITY_MAX_RAD_S (33.0F)
#define HBALL_RS00_TORQUE_MIN_NM (-14.0F)
#define HBALL_RS00_TORQUE_MAX_NM (14.0F)

typedef struct
{
    uint32_t id;
    uint8_t is_extended;
    uint8_t is_remote;
    uint8_t dlc;
    uint8_t data[8];
} hball_can_frame_t;

typedef struct
{
    uint8_t motor_id;
    uint8_t mode_state;
    uint8_t fault_summary;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float temperature_c;
} hball_motor_feedback_t;

typedef enum
{
    HBALL_CAN_EVENT_IGNORED = 0,
    HBALL_CAN_EVENT_INVALID,
    HBALL_CAN_EVENT_PROBE_REPLY,
    HBALL_CAN_EVENT_FEEDBACK,
} hball_can_event_t;

typedef struct
{
    uint8_t motor_id;
    bool probe_pending;
    bool probe_valid;
    bool feedback_valid;
    uint64_t unique_id;
    hball_motor_feedback_t feedback;
    uint32_t last_probe_ms;
    uint32_t last_feedback_ms;
    uint32_t rx_total;
    uint32_t rx_invalid;
    uint32_t rx_ignored;
} hball_motor_monitor_t;

typedef enum
{
    HBALL_MSP_EVENT_IGNORED = 0,
    HBALL_MSP_EVENT_INVALID,
    HBALL_MSP_EVENT_HEARTBEAT,
    HBALL_MSP_EVENT_ACCEL,
    HBALL_MSP_EVENT_GYRO,
    HBALL_MSP_EVENT_WHEEL,
    HBALL_MSP_EVENT_ATTITUDE,
    HBALL_MSP_EVENT_DUPLICATE,
    HBALL_MSP_EVENT_OUT_OF_ORDER,
} hball_msp_event_t;

typedef struct
{
    bool heartbeat_valid;
    bool accel_valid;
    bool gyro_valid;
    bool wheel_valid;
    bool attitude_valid;
    uint16_t heartbeat_sequence;
    uint16_t accel_sequence;
    uint16_t gyro_sequence;
    uint16_t wheel_sequence;
    uint16_t attitude_sequence;
    uint16_t status_flags;
    uint32_t uptime_ms;
    float accel_mps2[3];
    float gyro_rad_s[3];
    float attitude_rad[3];
    float wheel_left_mps;
    float wheel_right_mps;
    float body_speed_mps;
    uint32_t last_heartbeat_ms;
    uint32_t last_accel_ms;
    uint32_t last_gyro_ms;
    uint32_t last_wheel_ms;
    uint32_t last_attitude_ms;
    uint32_t rx_total;
    uint32_t rx_invalid;
    uint32_t rx_ignored;
    uint32_t rx_duplicate;
    uint32_t rx_out_of_order;
    uint32_t rx_gap;
    uint32_t reboot_total;
} hball_msp_monitor_t;

uint32_t hball_rs00_ext_id(uint8_t comm_type, uint16_t data2, uint8_t data1);
void hball_motor_monitor_init(hball_motor_monitor_t *monitor, uint8_t motor_id);
bool hball_motor_monitor_make_probe(
    hball_motor_monitor_t *monitor, hball_can_frame_t *frame
);
hball_can_event_t hball_motor_monitor_accept(
    hball_motor_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint32_t now_ms
);
bool hball_motor_monitor_feedback_fresh(
    const hball_motor_monitor_t *monitor,
    uint32_t now_ms,
    uint32_t timeout_ms
);
void hball_msp_monitor_init(hball_msp_monitor_t *monitor);
hball_msp_event_t hball_msp_monitor_accept(
    hball_msp_monitor_t *monitor,
    const hball_can_frame_t *frame,
    uint32_t now_ms
);
bool hball_msp_monitor_imu_fresh(
    const hball_msp_monitor_t *monitor,
    uint32_t now_ms,
    uint32_t timeout_ms
);
bool hball_msp_monitor_heartbeat_fresh(
    const hball_msp_monitor_t *monitor,
    uint32_t now_ms,
    uint32_t timeout_ms
);

#ifdef __cplusplus
}
#endif

#endif

#ifndef HBALL_MISSION_CAN_H
#define HBALL_MISSION_CAN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_MISSION_CAN_DLC 8U

#define HBALL_CAN_ID_MISSION_INTENT UINT32_C(0x081)
#define HBALL_CAN_ID_MISSION_STATUS UINT32_C(0x082)
#define HBALL_CAN_ID_MISSION_UI UINT32_C(0x083)
#define HBALL_CAN_ID_MISSION_CHASSIS_STATUS UINT32_C(0x084)

#define HBALL_MISSION_READY_M33_ALIVE (UINT16_C(1) << 0)
#define HBALL_MISSION_READY_MSP_LINK (UINT16_C(1) << 1)
#define HBALL_MISSION_READY_IMU (UINT16_C(1) << 2)
#define HBALL_MISSION_READY_CHASSIS (UINT16_C(1) << 3)
#define HBALL_MISSION_READY_TRACK (UINT16_C(1) << 4)
#define HBALL_MISSION_READY_PI_USB (UINT16_C(1) << 5)
#define HBALL_MISSION_READY_VISION (UINT16_C(1) << 6)
#define HBALL_MISSION_READY_RECORDER (UINT16_C(1) << 7)
#define HBALL_MISSION_READY_M55_ALIVE (UINT16_C(1) << 8)
#define HBALL_MISSION_READY_CONTROL (UINT16_C(1) << 9)
#define HBALL_MISSION_READY_RS00_LINK (UINT16_C(1) << 10)
#define HBALL_MISSION_READY_RS00_CONFIG (UINT16_C(1) << 11)
#define HBALL_MISSION_READY_SAFETY (UINT16_C(1) << 12)
#define HBALL_MISSION_READY_START_GEOMETRY (UINT16_C(1) << 13)
#define HBALL_MISSION_READY_BALL_PRECONDITION (UINT16_C(1) << 14)
#define HBALL_MISSION_READY_CONFIG_MATCH (UINT16_C(1) << 15)

#define HBALL_MISSION_CHASSIS_EVENT_LEFT_A (UINT8_C(1) << 0)
#define HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A (UINT8_C(1) << 1)
#define HBALL_MISSION_CHASSIS_EVENT_DETECTED_B (UINT8_C(1) << 2)
#define HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE (UINT8_C(1) << 3)
#define HBALL_MISSION_CHASSIS_EVENT_STOPPED (UINT8_C(1) << 4)
#define HBALL_MISSION_CHASSIS_EVENT_LINE_LOST (UINT8_C(1) << 5)
#define HBALL_MISSION_CHASSIS_EVENT_LOCAL_FAULT (UINT8_C(1) << 6)
#define HBALL_MISSION_CHASSIS_EVENT_INHIBITED (UINT8_C(1) << 7)

typedef enum
{
    HBALL_MISSION_Q2_FAST_LAP = 2,
    HBALL_MISSION_Q3_BALL_SEQUENCE = 3,
    HBALL_MISSION_Q4_A_TO_B = 4,
    HBALL_MISSION_Q5_CENTER_LAP = 5,
    HBALL_MISSION_Q6_HOLD_POSITION_LAP = 6,
} hball_mission_id_t;

typedef enum
{
    HBALL_MISSION_COMMAND_PREPARE = 1,
    HBALL_MISSION_COMMAND_START = 2,
    HBALL_MISSION_COMMAND_ABORT = 3,
    HBALL_MISSION_COMMAND_RESET = 4,
} hball_mission_command_t;

typedef enum
{
    HBALL_MISSION_STATE_BOOT = 0,
    HBALL_MISSION_STATE_SELF_TEST = 1,
    HBALL_MISSION_STATE_SELECT = 2,
    HBALL_MISSION_STATE_PREPARING = 3,
    HBALL_MISSION_STATE_READY = 4,
    HBALL_MISSION_STATE_START_PENDING = 5,
    HBALL_MISSION_STATE_RUNNING = 6,
    HBALL_MISSION_STATE_FINISHING = 7,
    HBALL_MISSION_STATE_COMPLETED = 8,
    HBALL_MISSION_STATE_CONTROLLED_ABORT = 9,
    HBALL_MISSION_STATE_FAULT_LATCHED = 10,
} hball_mission_state_t;

typedef enum
{
    HBALL_MISSION_REASON_NONE = 0,
    HBALL_MISSION_REASON_NOT_READY = 1,
    HBALL_MISSION_REASON_EPOCH_MISMATCH = 2,
    HBALL_MISSION_REASON_START_TIMEOUT = 3,
    HBALL_MISSION_REASON_LOCAL_FAULT = 4,
    HBALL_MISSION_REASON_LINK_FAULT = 5,
    HBALL_MISSION_REASON_CONFIG_MISMATCH = 6,
    HBALL_MISSION_REASON_DEADLINE = 7,
} hball_mission_reason_t;

typedef struct
{
    uint32_t id;
    uint8_t is_extended;
    uint8_t is_remote;
    uint8_t dlc;
    uint8_t data[HBALL_MISSION_CAN_DLC];
} hball_mission_can_frame_t;

typedef struct
{
    uint16_t epoch;
    uint8_t mission_id;
    uint8_t command;
    uint32_t event_time_ms;
} hball_mission_intent_t;

typedef struct
{
    uint16_t epoch;
    uint8_t mission_id;
    uint8_t global_state;
    uint16_t ready_mask;
    uint8_t reason;
    uint8_t status_sequence;
} hball_mission_status_t;

typedef struct
{
    uint16_t epoch;
    int16_t ball_position_mm;
    int16_t target_position_mm;
    uint8_t phase;
    uint8_t quality;
} hball_mission_ui_t;

typedef struct
{
    uint16_t epoch;
    uint8_t chassis_phase;
    uint8_t event_flags;
    uint32_t elapsed_ms;
} hball_mission_chassis_status_t;

bool hball_mission_id_valid(uint8_t mission_id);
bool hball_mission_encode_intent(
    const hball_mission_intent_t *message,
    hball_mission_can_frame_t *frame
);
bool hball_mission_decode_intent(
    const hball_mission_can_frame_t *frame,
    hball_mission_intent_t *message
);
bool hball_mission_encode_status(
    const hball_mission_status_t *message,
    hball_mission_can_frame_t *frame
);
bool hball_mission_decode_status(
    const hball_mission_can_frame_t *frame,
    hball_mission_status_t *message
);
bool hball_mission_encode_ui(
    const hball_mission_ui_t *message,
    hball_mission_can_frame_t *frame
);
bool hball_mission_decode_ui(
    const hball_mission_can_frame_t *frame,
    hball_mission_ui_t *message
);
bool hball_mission_encode_chassis_status(
    const hball_mission_chassis_status_t *message,
    hball_mission_can_frame_t *frame
);
bool hball_mission_decode_chassis_status(
    const hball_mission_can_frame_t *frame,
    hball_mission_chassis_status_t *message
);

#ifdef __cplusplus
}
#endif

#endif

#include "hball_mission_can.h"

#include <stddef.h>
#include <string.h>

static void hball_store_u16(uint8_t *target, uint16_t value)
{
    target[0] = (uint8_t)value;
    target[1] = (uint8_t)(value >> 8U);
}

static void hball_store_u32(uint8_t *target, uint32_t value)
{
    target[0] = (uint8_t)value;
    target[1] = (uint8_t)(value >> 8U);
    target[2] = (uint8_t)(value >> 16U);
    target[3] = (uint8_t)(value >> 24U);
}

static uint16_t hball_load_u16(const uint8_t *source)
{
    return (uint16_t)source[0] | ((uint16_t)source[1] << 8U);
}

static uint32_t hball_load_u32(const uint8_t *source)
{
    return (uint32_t)source[0]
        | ((uint32_t)source[1] << 8U)
        | ((uint32_t)source[2] << 16U)
        | ((uint32_t)source[3] << 24U);
}

static bool hball_frame_valid(
    const hball_mission_can_frame_t *frame, uint32_t expected_id
)
{
    return (frame != NULL)
        && (frame->id == expected_id)
        && (frame->is_extended == 0U)
        && (frame->is_remote == 0U)
        && (frame->dlc == HBALL_MISSION_CAN_DLC);
}

static void hball_frame_init(
    hball_mission_can_frame_t *frame, uint32_t id
)
{
    memset(frame, 0, sizeof(*frame));
    frame->id = id;
    frame->dlc = HBALL_MISSION_CAN_DLC;
}

bool hball_mission_id_valid(uint8_t mission_id)
{
    return (mission_id >= HBALL_MISSION_Q2_FAST_LAP)
        && (mission_id <= HBALL_MISSION_Q6_HOLD_POSITION_LAP);
}

bool hball_mission_encode_intent(
    const hball_mission_intent_t *message,
    hball_mission_can_frame_t *frame
)
{
    if ((message == NULL) || (frame == NULL)
        || (message->epoch == 0U)
        || !hball_mission_id_valid(message->mission_id)
        || (message->command < HBALL_MISSION_COMMAND_PREPARE)
        || (message->command > HBALL_MISSION_COMMAND_LEVEL))
    {
        return false;
    }

    hball_frame_init(frame, HBALL_CAN_ID_MISSION_INTENT);
    hball_store_u16(frame->data, message->epoch);
    frame->data[2] = message->mission_id;
    frame->data[3] = message->command;
    hball_store_u32(frame->data + 4U, message->event_time_ms);
    return true;
}

bool hball_mission_decode_intent(
    const hball_mission_can_frame_t *frame,
    hball_mission_intent_t *message
)
{
    hball_mission_intent_t decoded;

    if ((message == NULL)
        || !hball_frame_valid(frame, HBALL_CAN_ID_MISSION_INTENT))
    {
        return false;
    }
    decoded.epoch = hball_load_u16(frame->data);
    decoded.mission_id = frame->data[2];
    decoded.command = frame->data[3];
    decoded.event_time_ms = hball_load_u32(frame->data + 4U);
    if ((decoded.epoch == 0U)
        || !hball_mission_id_valid(decoded.mission_id)
        || (decoded.command < HBALL_MISSION_COMMAND_PREPARE)
        || (decoded.command > HBALL_MISSION_COMMAND_LEVEL))
    {
        return false;
    }
    *message = decoded;
    return true;
}
bool hball_mission_encode_status(
    const hball_mission_status_t *message,
    hball_mission_can_frame_t *frame
)
{
    if ((message == NULL) || (frame == NULL)
        || (message->epoch == 0U)
        || !hball_mission_id_valid(message->mission_id)
        || (message->global_state > HBALL_MISSION_STATE_FAULT_LATCHED)
        || (message->reason > HBALL_MISSION_REASON_DEADLINE))
    {
        return false;
    }

    hball_frame_init(frame, HBALL_CAN_ID_MISSION_STATUS);
    hball_store_u16(frame->data, message->epoch);
    frame->data[2] = message->mission_id;
    frame->data[3] = message->global_state;
    hball_store_u16(frame->data + 4U, message->ready_mask);
    frame->data[6] = message->reason;
    frame->data[7] = message->status_sequence;
    return true;
}

bool hball_mission_decode_status(
    const hball_mission_can_frame_t *frame,
    hball_mission_status_t *message
)
{
    hball_mission_status_t decoded;

    if ((message == NULL)
        || !hball_frame_valid(frame, HBALL_CAN_ID_MISSION_STATUS))
    {
        return false;
    }
    decoded.epoch = hball_load_u16(frame->data);
    decoded.mission_id = frame->data[2];
    decoded.global_state = frame->data[3];
    decoded.ready_mask = hball_load_u16(frame->data + 4U);
    decoded.reason = frame->data[6];
    decoded.status_sequence = frame->data[7];
    if ((decoded.epoch == 0U)
        || !hball_mission_id_valid(decoded.mission_id)
        || (decoded.global_state > HBALL_MISSION_STATE_FAULT_LATCHED)
        || (decoded.reason > HBALL_MISSION_REASON_DEADLINE))
    {
        return false;
    }
    *message = decoded;
    return true;
}

bool hball_mission_encode_ui(
    const hball_mission_ui_t *message,
    hball_mission_can_frame_t *frame
)
{
    if ((message == NULL) || (frame == NULL) || (message->epoch == 0U))
    {
        return false;
    }

    hball_frame_init(frame, HBALL_CAN_ID_MISSION_UI);
    hball_store_u16(frame->data, message->epoch);
    hball_store_u16(frame->data + 2U, (uint16_t)message->ball_position_mm);
    hball_store_u16(frame->data + 4U, (uint16_t)message->target_position_mm);
    frame->data[6] = message->phase;
    frame->data[7] = message->quality;
    return true;
}

bool hball_mission_decode_ui(
    const hball_mission_can_frame_t *frame,
    hball_mission_ui_t *message
)
{
    hball_mission_ui_t decoded;

    if ((message == NULL)
        || !hball_frame_valid(frame, HBALL_CAN_ID_MISSION_UI))
    {
        return false;
    }
    decoded.epoch = hball_load_u16(frame->data);
    decoded.ball_position_mm = (int16_t)hball_load_u16(frame->data + 2U);
    decoded.target_position_mm = (int16_t)hball_load_u16(frame->data + 4U);
    decoded.phase = frame->data[6];
    decoded.quality = frame->data[7];
    if (decoded.epoch == 0U)
    {
        return false;
    }
    *message = decoded;
    return true;
}

bool hball_mission_encode_chassis_status(
    const hball_mission_chassis_status_t *message,
    hball_mission_can_frame_t *frame
)
{
    if ((message == NULL) || (frame == NULL) || (message->epoch == 0U))
    {
        return false;
    }

    hball_frame_init(frame, HBALL_CAN_ID_MISSION_CHASSIS_STATUS);
    hball_store_u16(frame->data, message->epoch);
    frame->data[2] = message->chassis_phase;
    frame->data[3] = message->event_flags;
    hball_store_u32(frame->data + 4U, message->elapsed_ms);
    return true;
}

bool hball_mission_decode_chassis_status(
    const hball_mission_can_frame_t *frame,
    hball_mission_chassis_status_t *message
)
{
    hball_mission_chassis_status_t decoded;

    if ((message == NULL)
        || !hball_frame_valid(frame, HBALL_CAN_ID_MISSION_CHASSIS_STATUS))
    {
        return false;
    }
    decoded.epoch = hball_load_u16(frame->data);
    decoded.chassis_phase = frame->data[2];
    decoded.event_flags = frame->data[3];
    decoded.elapsed_ms = hball_load_u32(frame->data + 4U);
    if (decoded.epoch == 0U)
    {
        return false;
    }
    *message = decoded;
    return true;
}

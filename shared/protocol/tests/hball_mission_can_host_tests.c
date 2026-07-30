#include "hball_mission_can.h"

#include <assert.h>
#include <string.h>

static void assert_payload(
    const hball_mission_can_frame_t *frame,
    uint32_t id,
    const uint8_t expected[HBALL_MISSION_CAN_DLC]
)
{
    assert(frame->id == id);
    assert(frame->is_extended == 0U);
    assert(frame->is_remote == 0U);
    assert(frame->dlc == HBALL_MISSION_CAN_DLC);
    assert(memcmp(frame->data, expected, HBALL_MISSION_CAN_DLC) == 0);
}

static void test_intent_golden_vector_and_round_trip(void)
{
    const uint8_t expected[8] = {
        0x34U, 0x12U, 0x06U, 0x02U, 0x12U, 0x34U, 0x56U, 0x78U
    };
    const hball_mission_intent_t input = {
        0x1234U, HBALL_MISSION_Q6_HOLD_POSITION_LAP,
        HBALL_MISSION_COMMAND_START, UINT32_C(0x78563412)
    };
    hball_mission_intent_t output;
    hball_mission_can_frame_t frame;

    assert(hball_mission_encode_intent(&input, &frame));
    assert_payload(&frame, HBALL_CAN_ID_MISSION_INTENT, expected);
    memset(&output, 0, sizeof(output));
    assert(hball_mission_decode_intent(&frame, &output));
    assert(memcmp(&output, &input, sizeof(input)) == 0);
}

static void test_status_golden_vector_and_round_trip(void)
{
    const uint8_t expected[8] = {
        0x34U, 0x12U, 0x05U, 0x04U, 0x5aU, 0xa5U, 0x01U, 0x09U
    };
    const hball_mission_status_t input = {
        0x1234U, HBALL_MISSION_Q5_CENTER_LAP,
        HBALL_MISSION_STATE_READY, 0xa55aU,
        HBALL_MISSION_REASON_NOT_READY, 9U
    };
    hball_mission_status_t output;
    hball_mission_can_frame_t frame;

    assert(hball_mission_encode_status(&input, &frame));
    assert_payload(&frame, HBALL_CAN_ID_MISSION_STATUS, expected);
    memset(&output, 0, sizeof(output));
    assert(hball_mission_decode_status(&frame, &output));
    assert(memcmp(&output, &input, sizeof(input)) == 0);
}

static void test_ui_and_chassis_status_preserve_signed_and_time_values(void)
{
    const hball_mission_ui_t ui_input = {
        7U, -112, 50, 3U, 200U
    };
    const hball_mission_chassis_status_t chassis_input = {
        7U, 4U,
        HBALL_MISSION_CHASSIS_EVENT_LEFT_A
            | HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE,
        UINT32_C(0x01020304)
    };
    hball_mission_ui_t ui_output;
    hball_mission_chassis_status_t chassis_output;
    hball_mission_can_frame_t frame;

    assert(hball_mission_encode_ui(&ui_input, &frame));
    assert(frame.id == HBALL_CAN_ID_MISSION_UI);
    assert(hball_mission_decode_ui(&frame, &ui_output));
    assert(memcmp(&ui_output, &ui_input, sizeof(ui_input)) == 0);

    assert(hball_mission_encode_chassis_status(&chassis_input, &frame));
    assert(frame.id == HBALL_CAN_ID_MISSION_CHASSIS_STATUS);
    assert(hball_mission_decode_chassis_status(&frame, &chassis_output));
    assert(memcmp(&chassis_output, &chassis_input, sizeof(chassis_input)) == 0);
}

static void test_invalid_frames_and_values_fail_closed(void)
{
    hball_mission_can_frame_t frame;
    hball_mission_intent_t intent = {
        1U, HBALL_MISSION_Q2_FAST_LAP,
        HBALL_MISSION_COMMAND_PREPARE, 0U
    };
    hball_mission_status_t status = {
        1U, HBALL_MISSION_Q2_FAST_LAP,
        HBALL_MISSION_STATE_PREPARING, 0U,
        HBALL_MISSION_REASON_NONE, 0U
    };

    assert(hball_mission_encode_intent(&intent, &frame));
    frame.is_extended = 1U;
    assert(!hball_mission_decode_intent(&frame, &intent));
    frame.is_extended = 0U;
    frame.is_remote = 1U;
    assert(!hball_mission_decode_intent(&frame, &intent));
    frame.is_remote = 0U;
    frame.dlc = 7U;
    assert(!hball_mission_decode_intent(&frame, &intent));

    intent.epoch = 0U;
    assert(!hball_mission_encode_intent(&intent, &frame));
    intent.epoch = 1U;
    intent.mission_id = 1U;
    assert(!hball_mission_encode_intent(&intent, &frame));
    intent.mission_id = HBALL_MISSION_Q2_FAST_LAP;
    intent.command = 0xffU;
    assert(!hball_mission_encode_intent(&intent, &frame));

    status.global_state = 0xffU;
    assert(!hball_mission_encode_status(&status, &frame));
}

int main(void)
{
    test_intent_golden_vector_and_round_trip();
    test_status_golden_vector_and_round_trip();
    test_ui_and_chassis_status_preserve_signed_and_time_values();
    test_invalid_frames_and_values_fail_closed();
    return 0;
}

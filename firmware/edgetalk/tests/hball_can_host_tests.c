#include "hball_can.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static void test_get_id_frame_and_probe_reply(void)
{
    hball_motor_monitor_t monitor;
    hball_can_frame_t request;
    hball_can_frame_t reply;
    const uint8_t uid_bytes[8] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};

    hball_motor_monitor_init(&monitor, HBALL_RS00_MOTOR_ID);
    assert(hball_motor_monitor_make_probe(&monitor, &request));
    assert(request.id == 0x0000FD05UL);
    assert(request.is_extended == 1U);
    assert(request.is_remote == 0U);
    assert(request.dlc == 8U);

    memset(&reply, 0, sizeof(reply));
    reply.id = 0x000005FEUL;
    reply.is_extended = 1U;
    reply.dlc = 8U;
    memcpy(reply.data, uid_bytes, sizeof(uid_bytes));

    assert(hball_motor_monitor_accept(&monitor, &reply, 100U) == HBALL_CAN_EVENT_PROBE_REPLY);
    assert(monitor.probe_valid);
    assert(monitor.unique_id == UINT64_C(0x0807060504030201));
    assert(!monitor.probe_pending);
}

static void test_feedback_decode_and_freshness(void)
{
    hball_motor_monitor_t monitor;
    hball_can_frame_t frame;
    const uint16_t data2 = (uint16_t)((2U << 14) | (3U << 8) | HBALL_RS00_MOTOR_ID);

    hball_motor_monitor_init(&monitor, HBALL_RS00_MOTOR_ID);
    memset(&frame, 0, sizeof(frame));
    frame.id = hball_rs00_ext_id(HBALL_RS00_TYPE_FEEDBACK, data2, HBALL_RS00_MASTER_ID);
    frame.is_extended = 1U;
    frame.dlc = 8U;
    frame.data[0] = 0x80U;
    frame.data[1] = 0x00U;
    frame.data[2] = 0x80U;
    frame.data[3] = 0x00U;
    frame.data[4] = 0x80U;
    frame.data[5] = 0x00U;
    frame.data[6] = 0x00U;
    frame.data[7] = 0xFAU;

    assert(hball_motor_monitor_accept(&monitor, &frame, 1000U) == HBALL_CAN_EVENT_FEEDBACK);
    assert(monitor.feedback_valid);
    assert(monitor.feedback.motor_id == HBALL_RS00_MOTOR_ID);
    assert(monitor.feedback.mode_state == 2U);
    assert(monitor.feedback.fault_summary == 3U);
    assert(fabsf(monitor.feedback.position_rad) < 0.001F);
    assert(fabsf(monitor.feedback.velocity_rad_s) < 0.001F);
    assert(fabsf(monitor.feedback.torque_nm) < 0.001F);
    assert(fabsf(monitor.feedback.temperature_c - 25.0F) < 0.001F);
    assert(hball_motor_monitor_feedback_fresh(&monitor, 1049U, 50U));
    assert(!hball_motor_monitor_feedback_fresh(&monitor, 1051U, 50U));
}

static void test_invalid_or_wrong_host_frames_are_not_feedback(void)
{
    hball_motor_monitor_t monitor;
    hball_can_frame_t frame;

    hball_motor_monitor_init(&monitor, HBALL_RS00_MOTOR_ID);
    memset(&frame, 0, sizeof(frame));
    frame.id = hball_rs00_ext_id(HBALL_RS00_TYPE_FEEDBACK, HBALL_RS00_MOTOR_ID, 0xAAU);
    frame.is_extended = 1U;
    frame.dlc = 8U;
    assert(hball_motor_monitor_accept(&monitor, &frame, 1U) == HBALL_CAN_EVENT_IGNORED);

    frame.id = hball_rs00_ext_id(
        HBALL_RS00_TYPE_FEEDBACK, HBALL_RS00_MOTOR_ID, HBALL_RS00_MASTER_ID
    );
    frame.dlc = 4U;
    assert(hball_motor_monitor_accept(&monitor, &frame, 2U) == HBALL_CAN_EVENT_INVALID);
    assert(monitor.rx_total == 2U);
    assert(monitor.rx_invalid == 1U);
}

int main(void)
{
    test_get_id_frame_and_probe_reply();
    test_feedback_decode_and_freshness();
    test_invalid_or_wrong_host_frames_are_not_feedback();
    return 0;
}

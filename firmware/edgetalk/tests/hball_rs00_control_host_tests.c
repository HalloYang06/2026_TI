#include "hball_rs00_control.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static float hball_load_float_le(const uint8_t *data)
{
    const uint32_t bits = (uint32_t)data[0]
        | ((uint32_t)data[1] << 8U)
        | ((uint32_t)data[2] << 16U)
        | ((uint32_t)data[3] << 24U);
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void test_stop_and_enable_have_no_motion_payload(void)
{
    hball_can_frame_t frame;

    assert(hball_rs00_control_make_stop(HBALL_RS00_MOTOR_ID, false, &frame));
    assert(frame.id == 0x0400FD05UL);
    assert(frame.is_extended == 1U);
    assert(frame.dlc == 8U);
    for (uint8_t index = 0U; index < 8U; ++index)
    {
        assert(frame.data[index] == 0U);
    }

    assert(hball_rs00_control_make_enable(HBALL_RS00_MOTOR_ID, &frame));
    assert(frame.id == 0x0300FD05UL);
    for (uint8_t index = 0U; index < 8U; ++index)
    {
        assert(frame.data[index] == 0U);
    }
}

static void test_csp_mode_limits_and_position_are_exactly_encoded(void)
{
    hball_can_frame_t frame;

    assert(hball_rs00_control_make_csp_mode(HBALL_RS00_MOTOR_ID, &frame));
    assert(frame.id == 0x1200FD05UL);
    assert(frame.data[0] == 0x05U && frame.data[1] == 0x70U);
    assert(frame.data[4] == 5U);

    assert(hball_rs00_control_make_speed_limit(
        HBALL_RS00_MOTOR_ID, 0.5F, &frame
    ));
    assert(frame.data[0] == 0x17U && frame.data[1] == 0x70U);
    assert(fabsf(hball_load_float_le(frame.data + 4U) - 0.5F) < 1.0e-7F);

    assert(hball_rs00_control_make_current_limit(
        HBALL_RS00_MOTOR_ID, 0.8F, &frame
    ));
    assert(frame.data[0] == 0x18U && frame.data[1] == 0x70U);
    assert(fabsf(hball_load_float_le(frame.data + 4U) - 0.8F) < 1.0e-7F);

    assert(hball_rs00_control_make_position_reference(
        HBALL_RS00_MOTOR_ID, 1.887F, &frame
    ));
    assert(frame.data[0] == 0x16U && frame.data[1] == 0x70U);
    assert(fabsf(hball_load_float_le(frame.data + 4U) - 1.887F) < 1.0e-7F);
}

static void test_every_command_fails_closed_outside_bench_limits(void)
{
    hball_can_frame_t frame;

    assert(!hball_rs00_control_make_enable(0U, &frame));
    assert(!hball_rs00_control_make_speed_limit(
        HBALL_RS00_MOTOR_ID, 3.1F, &frame
    ));
    assert(!hball_rs00_control_make_current_limit(
        HBALL_RS00_MOTOR_ID, 2.1F, &frame
    ));
    assert(!hball_rs00_control_make_position_reference(
        HBALL_RS00_MOTOR_ID, NAN, &frame
    ));
    assert(!hball_rs00_control_make_position_reference(
        HBALL_RS00_MOTOR_ID, 13.0F, &frame
    ));
}

static void test_bench_state_machine_requires_mode_and_post_arm_feedback(void)
{
    hball_rs00_bench_t bench;

    hball_rs00_bench_init(&bench);
    assert(bench.state == HBALL_RS00_BENCH_SAFE);
    assert(hball_rs00_bench_begin_prepare(&bench, 1.887F, 10U));
    assert(bench.state == HBALL_RS00_BENCH_PREPARING);
    assert(!hball_rs00_bench_mark_prepared(&bench, 4U, 20U));
    assert(bench.state == HBALL_RS00_BENCH_FAULT);
    assert(bench.stop_reason == HBALL_RS00_BENCH_STOP_MODE_REJECTED);

    hball_rs00_bench_mark_stopped(
        &bench, HBALL_RS00_BENCH_STOP_MANUAL, 30U
    );
    assert(hball_rs00_bench_begin_prepare(&bench, 1.887F, 40U));
    assert(hball_rs00_bench_mark_prepared(
        &bench, HBALL_RS00_CSP_MODE, 50U
    ));
    assert(hball_rs00_bench_begin_arm(&bench, 60U));
    assert(bench.state == HBALL_RS00_BENCH_ARMING);
    assert(hball_rs00_bench_accept_feedback(&bench, 0U, 59U, 61U));
    assert(bench.state == HBALL_RS00_BENCH_ARMING);
    assert(hball_rs00_bench_accept_feedback(&bench, 0U, 60U, 62U));
    assert(bench.state == HBALL_RS00_BENCH_ARMED);
}

static void test_bench_step_is_tiny_bounded_and_returnable(void)
{
    hball_rs00_bench_t bench;
    float target = 0.0F;

    hball_rs00_bench_init(&bench);
    assert(hball_rs00_bench_begin_prepare(&bench, 1.887F, 0U));
    assert(hball_rs00_bench_mark_prepared(&bench, 5U, 1U));
    assert(hball_rs00_bench_begin_arm(&bench, 2U));
    assert(hball_rs00_bench_accept_feedback(&bench, 0U, 2U, 3U));
    assert(!hball_rs00_bench_make_step(&bench, 0.021F, 4U, &target));
    assert(hball_rs00_bench_make_step(&bench, 0.010F, 4U, &target));
    assert(fabsf(target - 1.897F) < 1.0e-6F);
    assert(bench.state == HBALL_RS00_BENCH_SMALL_STEP);
    assert(hball_rs00_bench_make_return(&bench, 5U, &target));
    assert(fabsf(target - 1.887F) < 1.0e-6F);
    assert(bench.state == HBALL_RS00_BENCH_RETURNING);
}

static void test_bench_watchdogs_and_motor_fault_fail_closed(void)
{
    hball_rs00_bench_t bench;

    hball_rs00_bench_init(&bench);
    assert(hball_rs00_bench_begin_prepare(&bench, 0.0F, 10U));
    assert(hball_rs00_bench_mark_prepared(&bench, 5U, 20U));
    assert(hball_rs00_bench_begin_arm(&bench, 30U));
    assert(!hball_rs00_bench_watchdog_expired(
        &bench, 30U + HBALL_RS00_BENCH_FEEDBACK_TIMEOUT_MS - 1U
    ));
    assert(hball_rs00_bench_watchdog_expired(
        &bench, 30U + HBALL_RS00_BENCH_FEEDBACK_TIMEOUT_MS
    ));
    assert(bench.stop_reason == HBALL_RS00_BENCH_STOP_FEEDBACK_TIMEOUT);

    hball_rs00_bench_mark_stopped(
        &bench, HBALL_RS00_BENCH_STOP_MANUAL, 200U
    );
    assert(hball_rs00_bench_begin_prepare(&bench, 0.0F, 300U));
    assert(hball_rs00_bench_mark_prepared(&bench, 5U, 301U));
    assert(hball_rs00_bench_begin_arm(&bench, 302U));
    assert(!hball_rs00_bench_accept_feedback(&bench, 1U, 302U, 303U));
    assert(bench.stop_reason == HBALL_RS00_BENCH_STOP_MOTOR_FAULT);
}

int main(void)
{
    test_stop_and_enable_have_no_motion_payload();
    test_csp_mode_limits_and_position_are_exactly_encoded();
    test_every_command_fails_closed_outside_bench_limits();
    test_bench_state_machine_requires_mode_and_post_arm_feedback();
    test_bench_step_is_tiny_bounded_and_returnable();
    test_bench_watchdogs_and_motor_fault_fail_closed();
    return 0;
}

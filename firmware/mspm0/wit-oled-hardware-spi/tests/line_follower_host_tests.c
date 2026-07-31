#include "line_follower.h"
#include "line_snapshot.h"

#include <assert.h>
#include <stdint.h>

static line_snapshot_t sample(uint8_t active_mask, uint32_t now_ms)
{
    return line_snapshot_decode((uint8_t)(~active_mask), now_ms);
}

static void test_q2_center_sample_preserves_verified_speed_mixing(void)
{
    line_follower_t follower;
    line_follower_output_t output;
    line_snapshot_t center_left = sample(UINT8_C(0x08), 10U);

    line_follower_init(&follower, LINE_FOLLOWER_PROFILE_Q2_FAST_LAP, 0U);
    assert(line_follower_step(&follower, &center_left, false, &output));
    assert(output.intent.valid);
    assert(output.intent.timestamp_ms == 10U);
    assert(output.intent.requested_speed_left == 60);
    assert(output.intent.requested_speed_right == 66);
    assert(output.intent.duty_slew_step == 3);
    assert(output.error == -5);
    assert(!output.reset_wheel_integrators);
    assert(!output.lost_timeout);
}

static void test_q2_lost_line_uses_normal_then_tight_recovery(void)
{
    line_follower_t follower;
    line_follower_output_t output;
    line_snapshot_t visible = sample(UINT8_C(0x08), 10U);
    line_snapshot_t lost = sample(0U, 20U);

    line_follower_init(&follower, LINE_FOLLOWER_PROFILE_Q2_FAST_LAP, 0U);
    assert(line_follower_step(&follower, &visible, false, &output));
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(output.intent.valid);
    assert(output.intent.timestamp_ms == 20U);
    assert(output.intent.requested_speed_left == 24);
    assert(output.intent.requested_speed_right == 52);
    assert(output.intent.duty_slew_step == 10);
    assert(!output.lost_timeout);

    lost = sample(0U, 320U);
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(output.intent.valid);
    assert(output.intent.timestamp_ms == 320U);
    assert(output.intent.requested_speed_left == 8);
    assert(output.intent.requested_speed_right == 44);
    assert(!output.lost_timeout);

    lost = sample(0U, 720U);
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(output.intent.valid);
    assert(output.intent.timestamp_ms == 720U);
    assert(output.intent.requested_speed_left == 8);
    assert(output.intent.requested_speed_right == 44);
    assert(!output.lost_timeout);

    visible = sample(UINT8_C(0x08), 730U);
    assert(line_follower_step(&follower, &visible, false, &output));
    assert(output.intent.requested_speed_left == 23);
    assert(output.intent.requested_speed_right == 58);
}

static void test_q2_curve_phase_keeps_confirm_and_exit_ramp_timing(void)
{
    line_follower_t follower;
    line_follower_output_t output;
    line_snapshot_t hard_left = sample(UINT8_C(0x01), 10U);
    line_snapshot_t center_left = sample(UINT8_C(0x08), 40U);

    line_follower_init(&follower, LINE_FOLLOWER_PROFILE_Q2_FAST_LAP, 0U);
    assert(line_follower_step(&follower, &hard_left, false, &output));
    assert(output.intent.requested_speed_left == 58);
    assert(output.intent.requested_speed_right == 68);
    assert(output.intent.duty_slew_step == 8);

    hard_left = sample(UINT8_C(0x01), 30U);
    assert(line_follower_step(&follower, &hard_left, false, &output));
    assert(output.intent.requested_speed_left == 53);
    assert(output.intent.requested_speed_right == 73);

    assert(line_follower_step(&follower, &center_left, false, &output));
    assert(output.intent.requested_speed_left == 52);
    assert(output.intent.requested_speed_right == 68);
    center_left = sample(UINT8_C(0x08), 120U);
    assert(line_follower_step(&follower, &center_left, false, &output));
    assert(output.intent.requested_speed_left == 52);
    assert(output.intent.requested_speed_right == 63);

    center_left = sample(UINT8_C(0x08), 130U);
    assert(line_follower_step(&follower, &center_left, false, &output));
    assert(output.intent.requested_speed_left == 52);
    assert(output.intent.requested_speed_right == 58);
    center_left = sample(UINT8_C(0x08), 370U);
    assert(line_follower_step(&follower, &center_left, false, &output));
    assert(output.intent.requested_speed_left == 57);
    assert(output.intent.requested_speed_right == 63);
}

static void test_force_straight_preserves_last_error_and_q2_duty_priority(void)
{
    line_follower_t follower;
    line_follower_output_t output;
    line_snapshot_t hard_left = sample(UINT8_C(0x01), 10U);
    line_snapshot_t wide = sample(UINT8_C(0xff), 20U);

    line_follower_init(&follower, LINE_FOLLOWER_PROFILE_Q2_FAST_LAP, 0U);
    assert(line_follower_step(&follower, &hard_left, false, &output));
    assert(line_follower_step(&follower, &wide, true, &output));
    assert(output.error == -35);
    assert(output.intent.requested_speed_left == 60);
    assert(output.intent.requested_speed_right == 66);
    assert(output.intent.duty_slew_step == 8);
}

static void test_stable_profile_reacquire_event_stays_latched_until_ack(void)
{
    line_follower_t follower;
    line_follower_output_t output;
    line_snapshot_t visible_right = sample(UINT8_C(0x10), 10U);
    line_snapshot_t lost = sample(0U, 20U);
    line_snapshot_t reacquired = sample(UINT8_C(0x10), 80U);
    line_snapshot_t next = sample(UINT8_C(0x18), 90U);

    line_follower_init(&follower, LINE_FOLLOWER_PROFILE_STABLE_LAP, 0U);
    assert(line_follower_step(&follower, &visible_right, false, &output));
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(output.intent.valid);
    assert(output.intent.timestamp_ms == 20U);
    lost = sample(0U, 70U);
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(output.intent.valid);
    assert(output.intent.timestamp_ms == 70U);
    assert(output.intent.requested_speed_left == 50);
    assert(output.intent.requested_speed_right == 20);

    assert(line_follower_step(&follower, &reacquired, false, &output));
    assert(output.reset_wheel_integrators);
    assert(output.line_reacquired_pending);
    assert(output.intent.duty_slew_step == 6);
    assert(line_follower_step(&follower, &next, false, &output));
    assert(!output.reset_wheel_integrators);
    assert(output.line_reacquired_pending);
    assert(output.intent.duty_slew_step == 6);

    line_follower_ack_motion_applied(&follower);
    next = sample(UINT8_C(0x18), 100U);
    assert(line_follower_step(&follower, &next, false, &output));
    assert(!output.line_reacquired_pending);
    assert(output.intent.duty_slew_step == 3);
}

static void test_stable_profile_holds_short_dropout_and_search_side(void)
{
    line_follower_t follower;
    line_follower_output_t output;
    line_snapshot_t visible_left = sample(UINT8_C(0x08), 10U);
    line_snapshot_t lost = sample(0U, 20U);
    line_snapshot_t visible_right = sample(UINT8_C(0x80), 80U);

    line_follower_init(&follower, LINE_FOLLOWER_PROFILE_STABLE_LAP, 0U);
    assert(line_follower_step(&follower, &visible_left, false, &output));
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(output.intent.requested_speed_left == 28);
    assert(output.intent.requested_speed_right == 28);

    lost = sample(0U, 60U);
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(output.intent.requested_speed_left == 20);
    assert(output.intent.requested_speed_right == 50);

    assert(line_follower_step(&follower, &visible_right, false, &output));
    lost = sample(0U, 90U);
    assert(line_follower_step(&follower, &lost, false, &output));
    lost = sample(0U, 130U);
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(output.intent.requested_speed_left == 20);
    assert(output.intent.requested_speed_right == 50);
}

static void test_stable_profile_preserves_integer_filter_and_preview(void)
{
    line_follower_t follower;
    line_follower_output_t output;
    line_snapshot_t center = sample(UINT8_C(0x18), 1000U);
    line_snapshot_t hard_left;
    uint32_t now_ms;

    line_follower_init(&follower, LINE_FOLLOWER_PROFILE_STABLE_LAP, 0U);
    for (now_ms = 1000U; now_ms <= 1050U; now_ms += 10U)
    {
        center = sample(UINT8_C(0x18), now_ms);
        assert(line_follower_step(&follower, &center, false, &output));
    }
    assert(output.intent.requested_speed_left == 46);
    assert(output.intent.requested_speed_right == 46);

    hard_left = sample(UINT8_C(0x01), 1060U);
    assert(line_follower_step(&follower, &hard_left, false, &output));
    assert(output.intent.requested_speed_left == 43);
    assert(output.intent.requested_speed_right == 45);
}

static void test_stable_profile_preserves_one_second_start_ramp(void)
{
    line_follower_t follower;
    line_follower_output_t output;
    line_snapshot_t center;
    uint32_t now_ms;

    line_follower_init(&follower, LINE_FOLLOWER_PROFILE_STABLE_LAP, 0U);
    for (now_ms = 0U; now_ms <= 500U; now_ms += 10U)
    {
        center = sample(UINT8_C(0x18), now_ms);
        assert(line_follower_step(&follower, &center, false, &output));
    }
    assert(output.intent.requested_speed_left == 37);
    assert(output.intent.requested_speed_right == 37);

    for (now_ms = 510U; now_ms <= 1000U; now_ms += 10U)
    {
        center = sample(UINT8_C(0x18), now_ms);
        assert(line_follower_step(&follower, &center, false, &output));
    }
    assert(output.intent.requested_speed_left == 46);
    assert(output.intent.requested_speed_right == 46);
}

static void test_q4_starts_at_full_profile_speed_on_center_line(void)
{
    line_follower_t follower;
    line_follower_output_t output;
    line_snapshot_t center = sample(UINT8_C(0x18), 10U);

    line_follower_init(&follower, LINE_FOLLOWER_PROFILE_Q4_TIMED_RUN, 0U);
    assert(line_follower_step(&follower, &center, false, &output));
    assert(output.intent.requested_speed_left == 50);
    assert(output.intent.requested_speed_right == 50);
    assert(output.intent.duty_slew_step == 3);

    center = sample(UINT8_C(0x01), 20U);
    assert(line_follower_step(&follower, &center, false, &output));
    assert(output.intent.requested_speed_left == 48);
    assert(output.intent.requested_speed_right == 52);
}

static void test_non_q2_profile_reports_lost_timeout_without_motion(void)
{
    line_follower_t follower;
    line_follower_output_t output;
    line_snapshot_t visible = sample(UINT8_C(0x18), 10U);
    line_snapshot_t lost = sample(0U, 20U);

    line_follower_init(&follower, LINE_FOLLOWER_PROFILE_Q4_TIMED_RUN, 0U);
    assert(line_follower_step(&follower, &visible, false, &output));
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(!output.lost_timeout);

    lost = sample(0U, 719U);
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(!output.lost_timeout);
    lost = sample(0U, 720U);
    assert(line_follower_step(&follower, &lost, false, &output));
    assert(output.lost_timeout);
    assert(!output.intent.valid);
}

int main(void)
{
    test_q2_center_sample_preserves_verified_speed_mixing();
    test_q2_lost_line_uses_normal_then_tight_recovery();
    test_q2_curve_phase_keeps_confirm_and_exit_ramp_timing();
    test_force_straight_preserves_last_error_and_q2_duty_priority();
    test_stable_profile_reacquire_event_stays_latched_until_ack();
    test_stable_profile_holds_short_dropout_and_search_side();
    test_stable_profile_preserves_integer_filter_and_preview();
    test_stable_profile_preserves_one_second_start_ramp();
    test_q4_starts_at_full_profile_speed_on_center_line();
    test_non_q2_profile_reports_lost_timeout_without_motion();
    return 0;
}

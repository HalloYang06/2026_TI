#include "hball_m33_q456.h"

#include <assert.h>
#include <math.h>

static void test_q4_q5_use_center_target(void)
{
    hball_m33_q456_t runtime;
    float target = 1.0F;

    hball_m33_q456_init(&runtime);
    assert(hball_m33_q456_sync_context(
        &runtime, 4U, HBALL_MISSION_Q4_A_TO_B));
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(target == 0.0F);

    assert(hball_m33_q456_sync_context(
        &runtime, 5U, HBALL_MISSION_Q5_CENTER_LAP));
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(target == 0.0F);
}

static void test_q6_target_steps_by_one_centimeter(void)
{
    hball_m33_q456_t runtime;
    float target;

    hball_m33_q456_init(&runtime);
    assert(hball_m33_q456_sync_context(
        &runtime, 6U, HBALL_MISSION_Q6_HOLD_POSITION_LAP));
    assert(!hball_m33_q456_start_target(&runtime, &target));
    assert(hball_m33_q456_step_q6_target(&runtime));
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(fabsf(target) < 1.0e-6F);

    assert(hball_m33_q456_step_q6_target(&runtime));
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(fabsf(target - 0.010F) < 1.0e-6F);
}

static void test_completion_requires_matching_epoch_and_events(void)
{
    hball_m33_q456_t runtime;
    float target;
    const uint8_t q4_done = HBALL_MISSION_CHASSIS_EVENT_STOPPED
        | HBALL_MISSION_CHASSIS_EVENT_DETECTED_B;

    hball_m33_q456_init(&runtime);
    assert(hball_m33_q456_sync_context(
        &runtime, 10U, HBALL_MISSION_Q4_A_TO_B));
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(hball_m33_q456_mark_running(&runtime, 100U));
    assert(hball_m33_q456_step(
        &runtime, 200U, 9U, q4_done, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        &runtime, 200U, 10U, q4_done, false)
        == HBALL_Q456_OUTCOME_COMPLETED);
}

static void test_deadlines_and_faults_are_explicit(void)
{
    hball_m33_q456_t runtime;
    float target;

    hball_m33_q456_init(&runtime);
    assert(hball_m33_q456_sync_context(
        &runtime, 11U, HBALL_MISSION_Q5_CENTER_LAP));
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(hball_m33_q456_mark_running(&runtime, 1000U));
    assert(hball_m33_q456_step(
        &runtime, 30999U, 11U, 0U, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        &runtime, 31000U, 11U, 0U, false)
        == HBALL_Q456_OUTCOME_DEADLINE);
    assert(hball_m33_q456_step(
        &runtime, 1001U, 11U, 0U, true)
        == HBALL_Q456_OUTCOME_CONTROL_FAULT);
}

static void start_q5(
    hball_m33_q456_t *runtime, uint16_t epoch, uint32_t now_ms
)
{
    float target;

    hball_m33_q456_init(runtime);
    assert(hball_m33_q456_sync_context(
        runtime, epoch, HBALL_MISSION_Q5_CENTER_LAP));
    assert(hball_m33_q456_start_target(runtime, &target));
    assert(hball_m33_q456_mark_running(runtime, now_ms));
}

static void start_q6(
    hball_m33_q456_t *runtime, uint16_t epoch, uint32_t now_ms
)
{
    float target;

    hball_m33_q456_init(runtime);
    assert(hball_m33_q456_sync_context(
        runtime, epoch, HBALL_MISSION_Q6_HOLD_POSITION_LAP));
    assert(hball_m33_q456_step_q6_target(runtime));
    assert(hball_m33_q456_start_target(runtime, &target));
    assert(hball_m33_q456_mark_running(runtime, now_ms));
}

static void test_q5_q6_completion_requires_ordered_route_events(void)
{
    hball_m33_q456_t runtime;
    const uint8_t active = HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE;
    const uint8_t left = HBALL_MISSION_CHASSIS_EVENT_LEFT_A;
    const uint8_t reacquired =
        HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A;
    const uint8_t stopped = HBALL_MISSION_CHASSIS_EVENT_STOPPED;

    start_q5(&runtime, 20U, 100U);
    assert(hball_m33_q456_step(
        &runtime, 101U, 20U, reacquired | stopped, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE);
    assert(hball_m33_q456_step(
        &runtime, 102U, 20U, left | reacquired | stopped, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE);
    assert(hball_m33_q456_step(
        &runtime, 103U, 20U, active, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_LEFT_A);
    assert(hball_m33_q456_step(
        &runtime, 104U, 20U, active | left, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_REACQUIRE_A);
    assert(hball_m33_q456_step(
        &runtime, 105U, 20U, active | left | stopped, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_REACQUIRE_A);
    assert(hball_m33_q456_step(
        &runtime, 106U, 20U, active | left | reacquired, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_STOPPED);
    assert(hball_m33_q456_step(
        &runtime, 107U, 20U, active | left | reacquired, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        &runtime, 108U, 20U, active | left | reacquired | stopped,
        false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        &runtime, 109U, 20U, left | reacquired | stopped, false)
        == HBALL_Q456_OUTCOME_COMPLETED);
}

static void test_q5_combined_events_advance_only_one_phase_per_step(void)
{
    hball_m33_q456_t runtime;
    const uint8_t all_events = HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE
        | HBALL_MISSION_CHASSIS_EVENT_LEFT_A
        | HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A
        | HBALL_MISSION_CHASSIS_EVENT_STOPPED;

    start_q5(&runtime, 21U, 100U);
    assert(hball_m33_q456_step(
        &runtime, 101U, 21U, all_events, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_LEFT_A);
    assert(hball_m33_q456_step(
        &runtime, 102U, 21U, all_events, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_REACQUIRE_A);
    assert(hball_m33_q456_step(
        &runtime, 103U, 21U, all_events, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_STOPPED);
    assert(hball_m33_q456_step(
        &runtime, 104U, 21U, all_events, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        &runtime, 105U, 21U,
        (uint8_t)(all_events
            & (uint8_t)~HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE),
        false)
        == HBALL_Q456_OUTCOME_COMPLETED);
}

static void test_q6_uses_the_same_ordered_route_phases(void)
{
    hball_m33_q456_t runtime;

    start_q6(&runtime, 24U, 100U);
    assert(hball_m33_q456_step(
        &runtime, 101U, 24U,
        HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A
            | HBALL_MISSION_CHASSIS_EVENT_STOPPED,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE);
    assert(hball_m33_q456_step(
        &runtime, 102U, 24U,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_LEFT_A);
    assert(hball_m33_q456_step(
        &runtime, 103U, 24U,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE
            | HBALL_MISSION_CHASSIS_EVENT_LEFT_A,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        &runtime, 104U, 24U,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE
            | HBALL_MISSION_CHASSIS_EVENT_LEFT_A
            | HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        &runtime, 105U, 24U,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE
            | HBALL_MISSION_CHASSIS_EVENT_LEFT_A
            | HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A
            | HBALL_MISSION_CHASSIS_EVENT_STOPPED,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        &runtime, 106U, 24U,
        HBALL_MISSION_CHASSIS_EVENT_LEFT_A
            | HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A
            | HBALL_MISSION_CHASSIS_EVENT_STOPPED,
        false) == HBALL_Q456_OUTCOME_COMPLETED);
}

static void test_q5_wrong_epoch_is_ignored_and_context_change_resets(void)
{
    hball_m33_q456_t runtime;
    float target;

    start_q5(&runtime, 22U, 100U);
    assert(hball_m33_q456_step(
        &runtime, 101U, 23U, HBALL_MISSION_CHASSIS_EVENT_LEFT_A,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE);

    assert(hball_m33_q456_step(
        &runtime, 102U, 22U,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_LEFT_A);

    assert(hball_m33_q456_step(
        &runtime, 103U, 22U,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE
            | HBALL_MISSION_CHASSIS_EVENT_LEFT_A,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_REACQUIRE_A);
    assert(hball_m33_q456_sync_context(
        &runtime, 23U, HBALL_MISSION_Q5_CENTER_LAP));
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE);
    assert(!runtime.running);
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(hball_m33_q456_mark_running(&runtime, 200U));
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE);
}

static void test_q5_same_epoch_prepare_resets_run_lifecycle(void)
{
    hball_m33_q456_t runtime;
    float target;
    const uint8_t left = HBALL_MISSION_CHASSIS_EVENT_LEFT_A;
    const uint8_t reacquired =
        HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A;

    start_q5(&runtime, 25U, 100U);
    assert(hball_m33_q456_step(
        &runtime, 101U, 25U,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        &runtime, 102U, 25U,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE | left, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        &runtime, 103U, 25U,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE | left
            | reacquired, false)
        == HBALL_Q456_OUTCOME_NONE);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_STOPPED);

    assert(hball_m33_q456_prepare(&runtime));
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE);
    assert(runtime.running_since_ms == 0U);
    assert(!runtime.running);
    assert(hball_m33_q456_sync_context(
        &runtime, 25U, HBALL_MISSION_Q5_CENTER_LAP));
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(hball_m33_q456_mark_running(&runtime, 1000U));
    assert(runtime.running);
    assert(runtime.running_since_ms == 1000U);
    assert(runtime.route_phase == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE);
    assert(hball_m33_q456_step(
        &runtime, 1001U, 25U, reacquired
            | HBALL_MISSION_CHASSIS_EVENT_STOPPED, false)
        == HBALL_Q456_OUTCOME_NONE);
}

static void advance_q5_to_wait_stopped(
    hball_m33_q456_t *runtime, uint16_t epoch
)
{
    assert(hball_m33_q456_step(
        runtime, 1U, epoch, HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        runtime, 2U, epoch,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE
            | HBALL_MISSION_CHASSIS_EVENT_LEFT_A,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(hball_m33_q456_step(
        runtime, 3U, epoch,
        HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE
            | HBALL_MISSION_CHASSIS_EVENT_LEFT_A
            | HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A,
        false) == HBALL_Q456_OUTCOME_NONE);
    assert(runtime->route_phase == HBALL_Q456_ROUTE_WAIT_STOPPED);
}

static void test_q6_prepare_relatches_target_on_same_epoch(void)
{
    hball_m33_q456_t runtime;
    float target;

    start_q6(&runtime, 30U, 100U);
    assert(hball_m33_q456_prepare(&runtime));
    assert(!runtime.running);
    assert(!runtime.target_latched);
    assert(runtime.sample_count == 0U);
    assert(!runtime.vision_sequence_valid);

    assert(hball_m33_q456_step_q6_target(&runtime));
    assert(hball_m33_q456_step_q6_target(&runtime));
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(fabsf(target - 0.010F) < 1.0e-6F);
}

static void test_q6_prepare_resets_target_latched_before_running(void)
{
    hball_m33_q456_t runtime;
    float target;

    hball_m33_q456_init(&runtime);
    assert(hball_m33_q456_sync_context(
        &runtime, 31U, HBALL_MISSION_Q6_HOLD_POSITION_LAP));
    assert(hball_m33_q456_prepare(&runtime));
    assert(hball_m33_q456_step_q6_target(&runtime));
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(runtime.target_latched);
    assert(!runtime.running);

    assert(hball_m33_q456_prepare(&runtime));
    assert(!runtime.target_latched);
    assert(runtime.sample_count == 0U);
    assert(runtime.sample_next == 0U);
    assert(!runtime.vision_sequence_valid);
    assert(!hball_m33_q456_start_target(&runtime, &target));
}

static void test_q5_matching_chassis_faults_precede_success(void)
{
    static const uint8_t faults[3] = {
        HBALL_MISSION_CHASSIS_EVENT_LOCAL_FAULT,
        HBALL_MISSION_CHASSIS_EVENT_LINE_LOST,
        HBALL_MISSION_CHASSIS_EVENT_INHIBITED,
    };
    const uint8_t success = HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE
        | HBALL_MISSION_CHASSIS_EVENT_LEFT_A
        | HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A
        | HBALL_MISSION_CHASSIS_EVENT_STOPPED;
    const uint8_t stale_success = HBALL_MISSION_CHASSIS_EVENT_LEFT_A
        | HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A
        | HBALL_MISSION_CHASSIS_EVENT_STOPPED;
    hball_m33_q456_t runtime;
    uint32_t index;

    for (index = 0U; index < 3U; ++index)
    {
        start_q5(&runtime, (uint16_t)(40U + index), 0U);
        assert(hball_m33_q456_step(
            &runtime, 1U, (uint16_t)(40U + index),
            (uint8_t)(stale_success | faults[index]), false)
            == HBALL_Q456_OUTCOME_NONE);
        assert(runtime.route_phase
            == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE);
        assert(hball_m33_q456_step(
            &runtime, 2U, (uint16_t)(40U + index),
            (uint8_t)(success | faults[index]), false)
            == HBALL_Q456_OUTCOME_CONTROL_FAULT);
        assert(runtime.route_phase
            == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE);
    }

    start_q5(&runtime, 43U, 0U);
    assert(hball_m33_q456_step(
        &runtime, 1U, 44U,
        HBALL_MISSION_CHASSIS_EVENT_LOCAL_FAULT, false)
        == HBALL_Q456_OUTCOME_NONE);
}

static void test_q5_deadline_precedes_success_at_boundary(void)
{
    hball_m33_q456_t runtime;

    start_q5(&runtime, 26U, 0U);
    advance_q5_to_wait_stopped(&runtime, 26U);
    assert(hball_m33_q456_step(
        &runtime, 29999U, 26U, HBALL_MISSION_CHASSIS_EVENT_STOPPED,
        false) == HBALL_Q456_OUTCOME_COMPLETED);

    start_q5(&runtime, 27U, 0U);
    advance_q5_to_wait_stopped(&runtime, 27U);
    assert(hball_m33_q456_step(
        &runtime, 30000U, 27U, HBALL_MISSION_CHASSIS_EVENT_STOPPED,
        false) == HBALL_Q456_OUTCOME_DEADLINE);

    start_q5(&runtime, 28U, 0U);
    advance_q5_to_wait_stopped(&runtime, 28U);
    assert(hball_m33_q456_step(
        &runtime, 30001U, 28U, HBALL_MISSION_CHASSIS_EVENT_STOPPED,
        false) == HBALL_Q456_OUTCOME_DEADLINE);

    start_q5(&runtime, 29U, 0U);
    advance_q5_to_wait_stopped(&runtime, 29U);
    assert(hball_m33_q456_step(
        &runtime, 30000U, 29U, HBALL_MISSION_CHASSIS_EVENT_STOPPED,
        true) == HBALL_Q456_OUTCOME_CONTROL_FAULT);
}

int main(void)
{
    test_q4_q5_use_center_target();
    test_q6_target_steps_by_one_centimeter();
    test_completion_requires_matching_epoch_and_events();
    test_deadlines_and_faults_are_explicit();
    test_q5_q6_completion_requires_ordered_route_events();
    test_q5_combined_events_advance_only_one_phase_per_step();
    test_q6_uses_the_same_ordered_route_phases();
    test_q5_wrong_epoch_is_ignored_and_context_change_resets();
    test_q5_same_epoch_prepare_resets_run_lifecycle();
    test_q6_prepare_relatches_target_on_same_epoch();
    test_q6_prepare_resets_target_latched_before_running();
    test_q5_matching_chassis_faults_precede_success();
    test_q5_deadline_precedes_success_at_boundary();
    return 0;
}

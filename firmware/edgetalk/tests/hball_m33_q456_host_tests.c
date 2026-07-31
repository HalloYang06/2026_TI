#include "hball_m33_q456.h"

#include <assert.h>
#include <math.h>

static void add_q6_samples(hball_m33_q456_t *runtime)
{
    static const float samples[5] = {
        0.020F, 0.018F, 0.060F, 0.019F, 0.017F
    };
    uint32_t index;

    for (index = 0U; index < 5U; ++index)
    {
        hball_m33_q456_observe_vision(
            runtime, index + 1U, samples[index], true
        );
    }
}

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

static void test_q6_latches_five_sample_median_once(void)
{
    hball_m33_q456_t runtime;
    float target;

    hball_m33_q456_init(&runtime);
    assert(hball_m33_q456_sync_context(
        &runtime, 6U, HBALL_MISSION_Q6_HOLD_POSITION_LAP));
    hball_m33_q456_observe_vision(&runtime, 1U, 0.020F, true);
    assert(!hball_m33_q456_start_target(&runtime, &target));
    add_q6_samples(&runtime);
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(fabsf(target - 0.019F) < 1.0e-6F);

    hball_m33_q456_observe_vision(&runtime, 20U, -0.070F, true);
    assert(hball_m33_q456_start_target(&runtime, &target));
    assert(fabsf(target - 0.019F) < 1.0e-6F);
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

int main(void)
{
    test_q4_q5_use_center_target();
    test_q6_latches_five_sample_median_once();
    test_completion_requires_matching_epoch_and_events();
    test_deadlines_and_faults_are_explicit();
    return 0;
}

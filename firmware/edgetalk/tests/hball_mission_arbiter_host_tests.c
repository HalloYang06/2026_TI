#include "hball_mission_arbiter.h"

#include <assert.h>

static hball_mission_intent_t make_intent(
    uint16_t epoch, uint8_t mission_id, uint8_t command
)
{
    const hball_mission_intent_t intent = {
        epoch, mission_id, command, 123U
    };
    return intent;
}

static void test_prepare_requires_complete_mask_for_500_ms(void)
{
    hball_mission_arbiter_t arbiter;
    hball_mission_intent_t intent = make_intent(
        1U, HBALL_MISSION_Q2_FAST_LAP, HBALL_MISSION_COMMAND_PREPARE
    );
    uint16_t required;

    hball_mission_arbiter_init(&arbiter);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &intent, 10U));
    assert(arbiter.global_state == HBALL_MISSION_STATE_PREPARING);
    required = hball_mission_required_ready_mask(intent.mission_id);
    assert((required & HBALL_MISSION_READY_CHASSIS) != 0U);
    assert((required & HBALL_MISSION_READY_IMU) == 0U);
    assert((required & HBALL_MISSION_READY_VISION) == 0U);
    assert((required & HBALL_MISSION_READY_RS00_LINK) == 0U);

    hball_mission_arbiter_update_ready(&arbiter, required, 100U);
    assert(arbiter.global_state == HBALL_MISSION_STATE_PREPARING);
    hball_mission_arbiter_update_ready(&arbiter, required, 599U);
    assert(arbiter.global_state == HBALL_MISSION_STATE_PREPARING);
    hball_mission_arbiter_update_ready(&arbiter, required, 600U);
    assert(arbiter.global_state == HBALL_MISSION_STATE_READY);

    hball_mission_arbiter_update_ready(
        &arbiter,
        (uint16_t)(required & ~HBALL_MISSION_READY_CHASSIS),
        601U
    );
    assert(arbiter.global_state == HBALL_MISSION_STATE_PREPARING);
    assert(arbiter.reason == HBALL_MISSION_REASON_NOT_READY);
}

static void test_start_outside_ready_is_rejected_and_not_queued(void)
{
    hball_mission_arbiter_t arbiter;
    hball_mission_intent_t prepare = make_intent(
        4U, HBALL_MISSION_Q4_A_TO_B, HBALL_MISSION_COMMAND_PREPARE
    );
    hball_mission_intent_t start = prepare;

    start.command = HBALL_MISSION_COMMAND_START;
    hball_mission_arbiter_init(&arbiter);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &prepare, 0U));
    assert(!hball_mission_arbiter_accept_intent(&arbiter, &start, 1U));
    assert(arbiter.global_state == HBALL_MISSION_STATE_PREPARING);
    assert(arbiter.start_reject_total == 1U);

    hball_mission_arbiter_update_ready(
        &arbiter, hball_mission_required_ready_mask(prepare.mission_id), 10U
    );
    hball_mission_arbiter_update_ready(
        &arbiter, hball_mission_required_ready_mask(prepare.mission_id), 510U
    );
    assert(arbiter.global_state == HBALL_MISSION_STATE_READY);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &start, 511U));
    assert(arbiter.global_state == HBALL_MISSION_STATE_START_PENDING);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &start, 512U));
    assert(arbiter.start_accept_total == 1U);
}

static void test_epoch_and_mission_must_match_after_start(void)
{
    hball_mission_arbiter_t arbiter;
    hball_mission_intent_t prepare = make_intent(
        9U, HBALL_MISSION_Q5_CENTER_LAP,
        HBALL_MISSION_COMMAND_PREPARE
    );
    hball_mission_intent_t wrong = prepare;

    hball_mission_arbiter_init(&arbiter);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &prepare, 0U));
    wrong.epoch = 8U;
    assert(!hball_mission_arbiter_accept_intent(&arbiter, &wrong, 1U));
    wrong.epoch = 9U;
    wrong.mission_id = HBALL_MISSION_Q6_HOLD_POSITION_LAP;
    assert(hball_mission_arbiter_accept_intent(&arbiter, &wrong, 2U));
    assert(arbiter.mission_id == HBALL_MISSION_Q6_HOLD_POSITION_LAP);
    assert(arbiter.epoch_reject_total == 1U);
}

static void test_q3_uses_available_stationary_dependencies(void)
{
    const uint16_t required = hball_mission_required_ready_mask(
        HBALL_MISSION_Q3_BALL_SEQUENCE
    );

    assert((required & HBALL_MISSION_READY_TRACK) == 0U);
    assert((required & HBALL_MISSION_READY_CHASSIS) != 0U);
    assert((required & HBALL_MISSION_READY_IMU) == 0U);
    assert((required & HBALL_MISSION_READY_VISION) != 0U);
}

static void test_q4_requires_mobile_ball_dependencies(void)
{
    const uint16_t required = hball_mission_required_ready_mask(
        HBALL_MISSION_Q4_A_TO_B
    );

    assert((required & HBALL_MISSION_READY_CHASSIS) != 0U);
    assert((required & HBALL_MISSION_READY_IMU) != 0U);
    assert((required & HBALL_MISSION_READY_VISION) != 0U);
    assert((required & HBALL_MISSION_READY_RS00_LINK) != 0U);
}

static void test_status_mirrors_context_and_increments_sequence(void)
{
    hball_mission_arbiter_t arbiter;
    hball_mission_status_t first;
    hball_mission_status_t second;
    hball_mission_intent_t prepare = make_intent(
        3U, HBALL_MISSION_Q3_BALL_SEQUENCE,
        HBALL_MISSION_COMMAND_PREPARE
    );

    hball_mission_arbiter_init(&arbiter);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &prepare, 0U));
    assert(hball_mission_arbiter_make_status(&arbiter, &first));
    assert(hball_mission_arbiter_make_status(&arbiter, &second));
    assert(first.epoch == 3U);
    assert(first.mission_id == HBALL_MISSION_Q3_BALL_SEQUENCE);
    assert(second.status_sequence == (uint8_t)(first.status_sequence + 1U));
}

static void test_execution_transitions_are_explicit_and_bounded(void)
{
    hball_mission_arbiter_t arbiter;
    hball_mission_intent_t prepare = make_intent(
        6U, HBALL_MISSION_Q4_A_TO_B, HBALL_MISSION_COMMAND_PREPARE
    );
    hball_mission_intent_t start = prepare;
    const uint16_t required = hball_mission_required_ready_mask(
        prepare.mission_id
    );

    start.command = HBALL_MISSION_COMMAND_START;
    hball_mission_arbiter_init(&arbiter);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &prepare, 0U));
    hball_mission_arbiter_update_ready(&arbiter, required, 1U);
    hball_mission_arbiter_update_ready(&arbiter, required, 501U);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &start, 502U));
    assert(hball_mission_arbiter_mark_running(&arbiter));
    assert(!hball_mission_arbiter_mark_running(&arbiter));
    assert(hball_mission_arbiter_mark_completed(&arbiter));
    assert(!hball_mission_arbiter_mark_completed(&arbiter));
}

static void test_repeated_reset_does_not_restart_ready_stabilization(void)
{
    hball_mission_arbiter_t arbiter;
    hball_mission_intent_t prepare = make_intent(
        7U, HBALL_MISSION_Q3_BALL_SEQUENCE,
        HBALL_MISSION_COMMAND_PREPARE
    );
    hball_mission_intent_t reset = prepare;
    const uint16_t required = hball_mission_required_ready_mask(
        prepare.mission_id
    );

    reset.command = HBALL_MISSION_COMMAND_RESET;
    hball_mission_arbiter_init(&arbiter);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &prepare, 0U));
    hball_mission_arbiter_update_ready(&arbiter, required, 100U);
    assert(arbiter.ready_candidate_valid);

    assert(hball_mission_arbiter_accept_intent(&arbiter, &reset, 150U));
    assert(hball_mission_arbiter_accept_intent(&arbiter, &reset, 400U));
    assert(arbiter.ready_candidate_valid);
    assert(arbiter.ready_candidate_time_ms == 100U);

    hball_mission_arbiter_update_ready(&arbiter, required, 600U);
    assert(arbiter.global_state == HBALL_MISSION_STATE_READY);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &reset, 650U));
    assert(arbiter.global_state == HBALL_MISSION_STATE_READY);
}

static void test_reset_establishes_context_after_m33_reboot(void)
{
    hball_mission_arbiter_t arbiter;
    hball_mission_intent_t reset = make_intent(
        8U, HBALL_MISSION_Q3_BALL_SEQUENCE,
        HBALL_MISSION_COMMAND_RESET
    );
    const uint16_t required = hball_mission_required_ready_mask(
        reset.mission_id
    );

    hball_mission_arbiter_init(&arbiter);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &reset, 10U));
    assert(arbiter.context_valid);
    assert(arbiter.epoch == reset.epoch);
    assert(arbiter.mission_id == reset.mission_id);

    hball_mission_arbiter_update_ready(&arbiter, required, 100U);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &reset, 200U));
    hball_mission_arbiter_update_ready(&arbiter, required, 600U);
    assert(arbiter.global_state == HBALL_MISSION_STATE_READY);
}

static void test_reset_recovers_from_old_running_context(void)
{
    hball_mission_arbiter_t arbiter;
    hball_mission_intent_t old_prepare = make_intent(
        9U, HBALL_MISSION_Q4_A_TO_B, HBALL_MISSION_COMMAND_PREPARE
    );
    hball_mission_intent_t old_start = old_prepare;
    hball_mission_intent_t reboot_reset = make_intent(
        1U, HBALL_MISSION_Q2_FAST_LAP, HBALL_MISSION_COMMAND_RESET
    );
    const uint16_t required = hball_mission_required_ready_mask(
        old_prepare.mission_id
    );

    old_start.command = HBALL_MISSION_COMMAND_START;
    hball_mission_arbiter_init(&arbiter);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &old_prepare, 0U));
    hball_mission_arbiter_update_ready(&arbiter, required, 1U);
    hball_mission_arbiter_update_ready(&arbiter, required, 501U);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &old_start, 502U));
    assert(hball_mission_arbiter_mark_running(&arbiter));

    assert(hball_mission_arbiter_accept_intent(
        &arbiter, &reboot_reset, 600U));
    assert(arbiter.global_state == HBALL_MISSION_STATE_CONTROLLED_ABORT);
    assert(arbiter.epoch == old_prepare.epoch);
    assert(arbiter.mission_id == old_prepare.mission_id);

    assert(hball_mission_arbiter_accept_intent(
        &arbiter, &reboot_reset, 650U));
    assert(arbiter.global_state == HBALL_MISSION_STATE_PREPARING);
    assert(arbiter.epoch == reboot_reset.epoch);
    assert(arbiter.mission_id == reboot_reset.mission_id);
}

static void test_same_context_reset_aborts_running_task(void)
{
    hball_mission_arbiter_t arbiter;
    hball_mission_intent_t prepare = make_intent(
        4U, HBALL_MISSION_Q3_BALL_SEQUENCE,
        HBALL_MISSION_COMMAND_PREPARE
    );
    hball_mission_intent_t start = prepare;
    hball_mission_intent_t reset = prepare;
    const uint16_t required = hball_mission_required_ready_mask(
        prepare.mission_id
    );

    start.command = HBALL_MISSION_COMMAND_START;
    reset.command = HBALL_MISSION_COMMAND_RESET;
    hball_mission_arbiter_init(&arbiter);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &prepare, 0U));
    hball_mission_arbiter_update_ready(&arbiter, required, 1U);
    hball_mission_arbiter_update_ready(&arbiter, required, 501U);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &start, 502U));
    assert(hball_mission_arbiter_mark_running(&arbiter));

    assert(hball_mission_arbiter_accept_intent(&arbiter, &reset, 600U));
    assert(arbiter.global_state == HBALL_MISSION_STATE_CONTROLLED_ABORT);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &reset, 650U));
    assert(arbiter.global_state == HBALL_MISSION_STATE_PREPARING);
}

static void test_start_rebuilds_context_after_m33_only_reboot(void)
{
    hball_mission_arbiter_t arbiter;
    hball_mission_intent_t start = make_intent(
        12U, HBALL_MISSION_Q3_BALL_SEQUENCE,
        HBALL_MISSION_COMMAND_START
    );
    const uint16_t required = hball_mission_required_ready_mask(
        start.mission_id
    );

    hball_mission_arbiter_init(&arbiter);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &start, 10U));
    assert(arbiter.context_valid);
    assert(arbiter.epoch == start.epoch);
    assert(arbiter.mission_id == start.mission_id);
    assert(arbiter.global_state == HBALL_MISSION_STATE_PREPARING);
    assert(arbiter.start_accept_total == 0U);

    hball_mission_arbiter_update_ready(&arbiter, required, 20U);
    hball_mission_arbiter_update_ready(&arbiter, required, 520U);
    assert(arbiter.global_state == HBALL_MISSION_STATE_READY);
    assert(hball_mission_arbiter_accept_intent(&arbiter, &start, 521U));
    assert(arbiter.global_state == HBALL_MISSION_STATE_START_PENDING);
    assert(arbiter.start_accept_total == 1U);
}

int main(void)
{
    test_prepare_requires_complete_mask_for_500_ms();
    test_start_outside_ready_is_rejected_and_not_queued();
    test_epoch_and_mission_must_match_after_start();
    test_q3_uses_available_stationary_dependencies();
    test_q4_requires_mobile_ball_dependencies();
    test_status_mirrors_context_and_increments_sequence();
    test_execution_transitions_are_explicit_and_bounded();
    test_repeated_reset_does_not_restart_ready_stabilization();
    test_reset_establishes_context_after_m33_reboot();
    test_reset_recovers_from_old_running_context();
    test_same_context_reset_aborts_running_task();
    test_start_rebuilds_context_after_m33_only_reboot();
    return 0;
}

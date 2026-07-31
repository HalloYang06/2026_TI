#include "hball_mission_client.h"

#include <assert.h>

static hball_mission_status_t make_status(
    uint16_t epoch,
    uint8_t mission_id,
    uint8_t state,
    uint8_t sequence
)
{
    hball_mission_status_t status = {
        epoch,
        mission_id,
        state,
        HBALL_MISSION_READY_M33_ALIVE | HBALL_MISSION_READY_MSP_LINK,
        state == HBALL_MISSION_STATE_READY
            ? HBALL_MISSION_REASON_NONE
            : HBALL_MISSION_REASON_NOT_READY,
        sequence,
    };
    return status;
}

static void test_client_starts_in_reset_and_requires_matching_ready(void)
{
    hball_mission_client_t client;
    hball_mission_intent_t intent;
    hball_mission_status_t status;

    hball_mission_client_init(&client, 10U);
    assert(client.candidate_epoch == 1U);
    assert(client.selected_mission == HBALL_MISSION_Q2_FAST_LAP);
    assert(!hball_mission_client_ready(&client, 10U));
    assert(hball_mission_client_make_intent(&client, &intent));
    assert(intent.command == HBALL_MISSION_COMMAND_RESET);

    status = make_status(1U, HBALL_MISSION_Q3_BALL_SEQUENCE,
                         HBALL_MISSION_STATE_READY, 1U);
    assert(!hball_mission_client_accept_status(&client, &status, 20U));
    assert(client.epoch_mismatch_total == 1U);

    status = make_status(1U, HBALL_MISSION_Q2_FAST_LAP,
                         HBALL_MISSION_STATE_PREPARING, 1U);
    assert(hball_mission_client_accept_status(&client, &status, 30U));
    assert(!hball_mission_client_ready(&client, 30U));

    status = make_status(1U, HBALL_MISSION_Q2_FAST_LAP,
                         HBALL_MISSION_STATE_READY, 2U);
    assert(hball_mission_client_accept_status(&client, &status, 40U));
    assert(hball_mission_client_ready(&client, 40U));
}

static void test_start_is_one_shot_and_keeps_button_time(void)
{
    hball_mission_client_t client;
    hball_mission_intent_t intent;
    hball_mission_status_t status;

    hball_mission_client_init(&client, 0U);
    status = make_status(1U, HBALL_MISSION_Q2_FAST_LAP,
                         HBALL_MISSION_STATE_READY, 1U);
    assert(hball_mission_client_accept_status(&client, &status, 100U));
    assert(hball_mission_client_request_start(&client, 120U));
    assert(!hball_mission_client_request_start(&client, 121U));
    assert(hball_mission_client_make_intent(&client, &intent));
    assert(intent.epoch == 1U);
    assert(intent.command == HBALL_MISSION_COMMAND_START);
    assert(intent.event_time_ms == 120U);
}

static void test_stale_duplicate_and_out_of_order_status_do_not_unlock(void)
{
    hball_mission_client_t client;
    hball_mission_status_t status;

    hball_mission_client_init(&client, 0U);
    status = make_status(1U, HBALL_MISSION_Q2_FAST_LAP,
                         HBALL_MISSION_STATE_READY, 10U);
    assert(hball_mission_client_accept_status(&client, &status, 100U));
    assert(hball_mission_client_ready(&client, 599U));
    assert(!hball_mission_client_ready(&client, 601U));

    assert(!hball_mission_client_accept_status(&client, &status, 500U));
    assert(client.duplicate_status_total == 1U);
    assert(hball_mission_client_ready(&client, 500U));

    status.status_sequence = 9U;
    assert(!hball_mission_client_accept_status(&client, &status, 510U));
    assert(client.out_of_order_status_total == 1U);
}

static void test_stale_sequence_rebases_after_m33_reboot(void)
{
    hball_mission_client_t client;
    hball_mission_status_t status;

    hball_mission_client_init(&client, 0U);
    status = make_status(1U, HBALL_MISSION_Q2_FAST_LAP,
                         HBALL_MISSION_STATE_READY, 200U);
    assert(hball_mission_client_accept_status(&client, &status, 100U));

    status.status_sequence = 1U;
    assert(!hball_mission_client_accept_status(&client, &status, 200U));
    assert(client.out_of_order_status_total == 1U);
    assert(hball_mission_client_accept_status(&client, &status, 601U));
    assert(client.status_resync_total == 1U);
    assert(hball_mission_client_ready(&client, 601U));
}

static void test_selection_is_allowed_without_status_but_blocked_after_start(void)
{
    hball_mission_client_t client;
    hball_mission_status_t status;

    hball_mission_client_init(&client, 0U);
    assert(hball_mission_client_select(
        &client, HBALL_MISSION_Q5_CENTER_LAP, 1U));
    assert(client.selected_mission == HBALL_MISSION_Q5_CENTER_LAP);
    assert(client.candidate_epoch == 2U);
    assert(!client.status_valid);

    status = make_status(2U, HBALL_MISSION_Q5_CENTER_LAP,
                         HBALL_MISSION_STATE_READY, 1U);
    assert(hball_mission_client_accept_status(&client, &status, 10U));
    assert(hball_mission_client_request_start(&client, 20U));
    assert(!hball_mission_client_select(
        &client, HBALL_MISSION_Q6_HOLD_POSITION_LAP, 21U));

    status = make_status(2U, HBALL_MISSION_Q5_CENTER_LAP,
                         HBALL_MISSION_STATE_COMPLETED, 2U);
    assert(hball_mission_client_accept_status(&client, &status, 30U));
    assert(!client.start_requested);
    assert(client.command == HBALL_MISSION_COMMAND_RESET);
    assert(hball_mission_client_select(
        &client, HBALL_MISSION_Q6_HOLD_POSITION_LAP, 31U));
    assert(client.candidate_epoch == 3U);
}

int main(void)
{
    test_client_starts_in_reset_and_requires_matching_ready();
    test_start_is_one_shot_and_keeps_button_time();
    test_stale_duplicate_and_out_of_order_status_do_not_unlock();
    test_stale_sequence_rebases_after_m33_reboot();
    test_selection_is_allowed_without_status_but_blocked_after_start();
    return 0;
}

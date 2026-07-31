#include "hball_mission_menu.h"

#include <assert.h>
#include <string.h>

static hball_mission_status_t make_status(
    uint8_t mission_id,
    uint8_t state,
    uint16_t ready_mask,
    uint8_t sequence
)
{
    hball_mission_status_t status = {
        1U,
        mission_id,
        state,
        ready_mask,
        state == HBALL_MISSION_STATE_READY
            ? HBALL_MISSION_REASON_NONE
            : HBALL_MISSION_REASON_NOT_READY,
        sequence,
    };
    return status;
}

static void test_selection_cycles_only_official_scoring_missions(void)
{
    assert(hball_mission_menu_next(HBALL_MISSION_Q2_FAST_LAP)
           == HBALL_MISSION_Q3_BALL_SEQUENCE);
    assert(hball_mission_menu_next(HBALL_MISSION_Q3_BALL_SEQUENCE)
           == HBALL_MISSION_Q4_A_TO_B);
    assert(hball_mission_menu_next(HBALL_MISSION_Q4_A_TO_B)
           == HBALL_MISSION_Q5_CENTER_LAP);
    assert(hball_mission_menu_next(HBALL_MISSION_Q5_CENTER_LAP)
           == HBALL_MISSION_Q6_HOLD_POSITION_LAP);
    assert(hball_mission_menu_next(HBALL_MISSION_Q6_HOLD_POSITION_LAP)
           == HBALL_MISSION_Q2_FAST_LAP);
}

static void test_execute_is_blocked_until_matching_status_is_ready(void)
{
    hball_mission_client_t client;
    hball_mission_status_t status;

    hball_mission_client_init(&client, 0U);
    assert(hball_mission_menu_handle(
               &client, HBALL_MISSION_MENU_EXECUTE, 10U)
           == HBALL_MISSION_MENU_START_BLOCKED);
    assert(!client.start_requested);

    status = make_status(
        HBALL_MISSION_Q2_FAST_LAP,
        HBALL_MISSION_STATE_PREPARING,
        HBALL_MISSION_READY_M33_ALIVE,
        1U
    );
    assert(hball_mission_client_accept_status(&client, &status, 20U));
    assert(hball_mission_menu_handle(
               &client, HBALL_MISSION_MENU_EXECUTE, 21U)
           == HBALL_MISSION_MENU_START_BLOCKED);
    assert(!client.start_requested);

    status = make_status(
        HBALL_MISSION_Q2_FAST_LAP,
        HBALL_MISSION_STATE_READY,
        UINT16_MAX,
        2U
    );
    assert(hball_mission_client_accept_status(&client, &status, 30U));
    assert(hball_mission_menu_handle(
               &client, HBALL_MISSION_MENU_EXECUTE, 31U)
           == HBALL_MISSION_MENU_START_ACCEPTED);
    assert(client.start_requested);
    assert(client.command == HBALL_MISSION_COMMAND_START);
}

static void test_selection_is_locked_after_start(void)
{
    hball_mission_client_t client;
    hball_mission_status_t status;

    hball_mission_client_init(&client, 0U);
    status = make_status(
        HBALL_MISSION_Q2_FAST_LAP,
        HBALL_MISSION_STATE_READY,
        UINT16_MAX,
        1U
    );
    assert(hball_mission_client_accept_status(&client, &status, 10U));
    assert(hball_mission_menu_handle(
               &client, HBALL_MISSION_MENU_EXECUTE, 11U)
           == HBALL_MISSION_MENU_START_ACCEPTED);
    assert(hball_mission_menu_handle(
               &client, HBALL_MISSION_MENU_SELECT, 12U)
           == HBALL_MISSION_MENU_LOCKED);
    assert(client.selected_mission == HBALL_MISSION_Q2_FAST_LAP);
}

static void test_display_reports_stale_status_and_first_missing_ready_bit(void)
{
    hball_mission_client_t client;
    hball_mission_status_t status;

    hball_mission_client_init(&client, 0U);
    assert(strcmp(hball_mission_menu_missing_label(&client, 0U),
                  "M33 STATUS") == 0);

    status = make_status(
        HBALL_MISSION_Q2_FAST_LAP,
        HBALL_MISSION_STATE_PREPARING,
        HBALL_MISSION_READY_M33_ALIVE
            | HBALL_MISSION_READY_MSP_LINK
            | HBALL_MISSION_READY_IMU,
        1U
    );
    assert(hball_mission_client_accept_status(&client, &status, 10U));
    assert(strcmp(hball_mission_menu_missing_label(&client, 10U),
                  "CHASSIS") == 0);
    assert(strcmp(hball_mission_menu_missing_label(&client, 161U),
                  "M33 STATUS") == 0);
}

static void test_labels_are_short_and_use_official_question_numbers(void)
{
    assert(strcmp(hball_mission_menu_mission_label(
                      HBALL_MISSION_Q2_FAST_LAP),
                  "Q2 FAST LAP") == 0);
    assert(strcmp(hball_mission_menu_mission_label(
                      HBALL_MISSION_Q6_HOLD_POSITION_LAP),
                  "Q6 HOLD X") == 0);
    assert(strcmp(hball_mission_menu_state_label(
                      HBALL_MISSION_STATE_PREPARING),
                  "PREPARING") == 0);
    assert(strcmp(hball_mission_menu_state_label(
                      HBALL_MISSION_STATE_RUNNING),
                  "RUNNING") == 0);
}

static void test_view_changes_only_when_displayed_content_changes(void)
{
    hball_mission_client_t client;
    hball_mission_status_t status;
    hball_mission_menu_view_t first;
    hball_mission_menu_view_t same;
    hball_mission_menu_view_t stale;

    hball_mission_client_init(&client, 0U);
    status = make_status(
        HBALL_MISSION_Q2_FAST_LAP,
        HBALL_MISSION_STATE_PREPARING,
        HBALL_MISSION_READY_M33_ALIVE,
        1U
    );
    assert(hball_mission_client_accept_status(&client, &status, 10U));
    assert(hball_mission_menu_make_view(&client, 10U, &first));
    assert(hball_mission_menu_make_view(&client, 100U, &same));
    assert(hball_mission_menu_view_equal(&first, &same));

    assert(hball_mission_menu_make_view(&client, 161U, &stale));
    assert(!hball_mission_menu_view_equal(&first, &stale));
    assert(strcmp(stale.state_label, "NO STATUS") == 0);
    assert(strcmp(stale.missing_label, "M33 STATUS") == 0);
}

int main(void)
{
    test_selection_cycles_only_official_scoring_missions();
    test_execute_is_blocked_until_matching_status_is_ready();
    test_selection_is_locked_after_start();
    test_display_reports_stale_status_and_first_missing_ready_bit();
    test_labels_are_short_and_use_official_question_numbers();
    test_view_changes_only_when_displayed_content_changes();
    return 0;
}

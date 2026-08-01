#include "hball_mission_menu.h"
#include "hball_mission_policy.h"

#include <stddef.h>
#include <string.h>

uint8_t hball_mission_menu_next(uint8_t mission_id)
{
    if ((mission_id < HBALL_MISSION_Q2_FAST_LAP)
        || (mission_id >= HBALL_MISSION_Q6_HOLD_POSITION_LAP))
    {
        return HBALL_MISSION_Q2_FAST_LAP;
    }
    return (uint8_t)(mission_id + 1U);
}

hball_mission_menu_result_t hball_mission_menu_handle(
    hball_mission_client_t *client,
    hball_mission_menu_event_t event,
    uint32_t now_ms
)
{
    if ((client == NULL) || (event == HBALL_MISSION_MENU_NONE))
    {
        return HBALL_MISSION_MENU_NO_CHANGE;
    }
    if (client->start_requested)
    {
        return HBALL_MISSION_MENU_LOCKED;
    }
    if (event == HBALL_MISSION_MENU_SELECT)
    {
        return hball_mission_client_select(
            client,
            hball_mission_menu_next(client->selected_mission),
            now_ms
        ) ? HBALL_MISSION_MENU_SELECTED : HBALL_MISSION_MENU_LOCKED;
    }
    if (event == HBALL_MISSION_MENU_EXECUTE)
    {
        hball_mission_policy_t policy;

        if (!hball_mission_policy_get(client->selected_mission, &policy))
        {
            return HBALL_MISSION_MENU_START_BLOCKED;
        }
        if (policy.local_start)
        {
            return HBALL_MISSION_MENU_LOCAL_START_ACCEPTED;
        }
        return hball_mission_client_request_start(client, now_ms)
            ? HBALL_MISSION_MENU_START_ACCEPTED
            : HBALL_MISSION_MENU_START_BLOCKED;
    }
    return HBALL_MISSION_MENU_NO_CHANGE;
}

uint16_t hball_mission_menu_required_mask(uint8_t mission_id)
{
    uint16_t required =
        HBALL_MISSION_READY_M33_ALIVE
        | HBALL_MISSION_READY_MSP_LINK
        | HBALL_MISSION_READY_CHASSIS
        | HBALL_MISSION_READY_PI_USB
        | HBALL_MISSION_READY_VISION
        | HBALL_MISSION_READY_RS00_LINK;

    if (mission_id != HBALL_MISSION_Q3_BALL_SEQUENCE)
    {
        required |= HBALL_MISSION_READY_IMU;
    }
    if ((mission_id >= HBALL_MISSION_Q3_BALL_SEQUENCE)
        && (mission_id <= HBALL_MISSION_Q5_CENTER_LAP))
    {
        required |= HBALL_MISSION_READY_START_GEOMETRY;
    }
    else if (mission_id == HBALL_MISSION_Q6_HOLD_POSITION_LAP)
    {
        required |= HBALL_MISSION_READY_BALL_PRECONDITION;
    }
    return required;
}

const char *hball_mission_menu_mission_label(uint8_t mission_id)
{
    switch (mission_id)
    {
    case HBALL_MISSION_Q2_FAST_LAP:
        return "Q2 FAST LAP";
    case HBALL_MISSION_Q3_BALL_SEQUENCE:
        return "Q3 BALL +/-5";
    case HBALL_MISSION_Q4_A_TO_B:
        return "Q4 A TO B";
    case HBALL_MISSION_Q5_CENTER_LAP:
        return "Q5 CENTER";
    case HBALL_MISSION_Q6_HOLD_POSITION_LAP:
        return "Q6 HOLD X";
    default:
        return "Q? INVALID";
    }
}

const char *hball_mission_menu_state_label(uint8_t global_state)
{
    switch (global_state)
    {
    case HBALL_MISSION_STATE_BOOT:
        return "BOOT";
    case HBALL_MISSION_STATE_SELF_TEST:
        return "SELF TEST";
    case HBALL_MISSION_STATE_SELECT:
        return "SELECT";
    case HBALL_MISSION_STATE_PREPARING:
        return "PREPARING";
    case HBALL_MISSION_STATE_READY:
        return "READY";
    case HBALL_MISSION_STATE_START_PENDING:
        return "START WAIT";
    case HBALL_MISSION_STATE_RUNNING:
        return "RUNNING";
    case HBALL_MISSION_STATE_FINISHING:
        return "FINISHING";
    case HBALL_MISSION_STATE_COMPLETED:
        return "COMPLETED";
    case HBALL_MISSION_STATE_CONTROLLED_ABORT:
        return "ABORTED";
    case HBALL_MISSION_STATE_FAULT_LATCHED:
        return "FAULT";
    default:
        return "NO STATUS";
    }
}

static const char *hball_mission_menu_missing_bit_label(uint16_t bit)
{
    static const char *const labels[16] = {
        "M33", "MSP LINK", "IMU", "CHASSIS",
        "TRACK", "PI USB", "VISION", "RECORDER",
        "M55", "CONTROL", "RS00 LINK", "RS00 CFG",
        "SAFETY", "START POS", "BALL POS", "CONFIG",
    };
    uint8_t index;

    for (index = 0U; index < 16U; ++index)
    {
        if ((bit & (uint16_t)(UINT16_C(1) << index)) != 0U)
        {
            return labels[index];
        }
    }
    return "NONE";
}

const char *hball_mission_menu_missing_label(
    const hball_mission_client_t *client, uint32_t now_ms
)
{
    hball_mission_policy_t policy;
    uint16_t missing;

    if ((client != NULL)
        && hball_mission_policy_get(client->selected_mission, &policy)
        && policy.local_start)
    {
        return "MSP LOCAL";
    }
    if ((client == NULL)
        || !client->status_valid
        || ((uint32_t)(now_ms - client->last_status_ms)
            > HBALL_MISSION_STATUS_FRESH_MS))
    {
        return "M33 STATUS";
    }
    missing = (uint16_t)(
        hball_mission_menu_required_mask(client->selected_mission)
        & ~client->latest_status.ready_mask
    );
    if (missing != 0U)
    {
        return hball_mission_menu_missing_bit_label(missing);
    }
    if (client->latest_status.global_state != HBALL_MISSION_STATE_READY)
    {
        return "STABILIZE";
    }
    return "ALL READY";
}

bool hball_mission_menu_make_view(
    const hball_mission_client_t *client,
    uint32_t now_ms,
    hball_mission_menu_view_t *view
)
{
    hball_mission_policy_t policy;

    if ((client == NULL) || (view == NULL))
    {
        return false;
    }
    memset(view, 0, sizeof(*view));
    view->mission_label = hball_mission_menu_mission_label(
        client->selected_mission
    );
    view->mission_id = client->selected_mission;
    view->state_label = "NO STATUS";
    view->missing_label = hball_mission_menu_missing_label(client, now_ms);
    view->epoch = client->candidate_epoch;
    view->start_requested = client->start_requested;
    view->local_execution =
        hball_mission_policy_get(client->selected_mission, &policy)
        && policy.local_start;
    view->status_fresh = client->status_valid
        && ((uint32_t)(now_ms - client->last_status_ms)
            <= HBALL_MISSION_STATUS_FRESH_MS);
    if (view->local_execution)
    {
        view->global_state = HBALL_MISSION_STATE_READY;
        view->state_label = "LOCAL READY";
    }
    else if (view->status_fresh)
    {
        view->global_state = client->latest_status.global_state;
        view->state_label = hball_mission_menu_state_label(
            view->global_state
        );
        view->ready_mask = client->latest_status.ready_mask;
    }
    return true;
}

bool hball_mission_menu_view_equal(
    const hball_mission_menu_view_t *left,
    const hball_mission_menu_view_t *right
)
{
    if ((left == NULL) || (right == NULL))
    {
        return false;
    }
    return (left->epoch == right->epoch)
        && (left->mission_id == right->mission_id)
        && (left->ready_mask == right->ready_mask)
        && (left->global_state == right->global_state)
        && (left->local_execution == right->local_execution)
        && (left->status_fresh == right->status_fresh)
        && (left->start_requested == right->start_requested)
        && (strcmp(left->mission_label, right->mission_label) == 0)
        && (strcmp(left->state_label, right->state_label) == 0)
        && (strcmp(left->missing_label, right->missing_label) == 0);
}

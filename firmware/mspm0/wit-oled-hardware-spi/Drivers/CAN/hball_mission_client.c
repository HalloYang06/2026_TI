#include "hball_mission_client.h"

#include <stddef.h>
#include <string.h>

void hball_mission_client_init(
    hball_mission_client_t *client, uint32_t now_ms
)
{
    if (client == NULL)
    {
        return;
    }
    memset(client, 0, sizeof(*client));
    client->candidate_epoch = 1U;
    client->selected_mission = HBALL_MISSION_Q2_FAST_LAP;
    client->command = HBALL_MISSION_COMMAND_PREPARE;
    client->command_time_ms = now_ms;
}

bool hball_mission_client_select(
    hball_mission_client_t *client,
    uint8_t mission_id,
    uint32_t now_ms
)
{
    if ((client == NULL)
        || client->start_requested
        || !hball_mission_id_valid(mission_id))
    {
        return false;
    }
    client->selected_mission = mission_id;
    client->command = HBALL_MISSION_COMMAND_PREPARE;
    client->command_time_ms = now_ms;
    client->status_valid = false;
    return true;
}

bool hball_mission_client_accept_status(
    hball_mission_client_t *client,
    const hball_mission_status_t *status,
    uint32_t now_ms
)
{
    uint8_t sequence_delta;

    if ((client == NULL) || (status == NULL))
    {
        return false;
    }
    if ((status->epoch != client->candidate_epoch)
        || (status->mission_id != client->selected_mission))
    {
        client->epoch_mismatch_total++;
        return false;
    }
    if (client->status_valid)
    {
        sequence_delta = (uint8_t)(
            status->status_sequence - client->latest_status.status_sequence
        );
        if (sequence_delta == 0U)
        {
            client->duplicate_status_total++;
            return false;
        }
        if (sequence_delta >= 128U)
        {
            client->out_of_order_status_total++;
            return false;
        }
    }

    client->latest_status = *status;
    client->last_status_ms = now_ms;
    client->status_valid = true;
    client->accepted_status_total++;
    return true;
}

bool hball_mission_client_ready(
    const hball_mission_client_t *client, uint32_t now_ms
)
{
    return (client != NULL)
        && client->status_valid
        && !client->start_requested
        && (client->latest_status.global_state == HBALL_MISSION_STATE_READY)
        && ((uint32_t)(now_ms - client->last_status_ms)
            <= HBALL_MISSION_STATUS_FRESH_MS);
}

bool hball_mission_client_request_start(
    hball_mission_client_t *client, uint32_t now_ms
)
{
    if ((client == NULL) || !hball_mission_client_ready(client, now_ms))
    {
        return false;
    }
    client->command = HBALL_MISSION_COMMAND_START;
    client->command_time_ms = now_ms;
    client->start_requested = true;
    return true;
}

bool hball_mission_client_make_intent(
    const hball_mission_client_t *client,
    hball_mission_intent_t *intent
)
{
    if ((client == NULL) || (intent == NULL))
    {
        return false;
    }
    intent->epoch = client->candidate_epoch;
    intent->mission_id = client->selected_mission;
    intent->command = client->command;
    intent->event_time_ms = client->command_time_ms;
    return true;
}

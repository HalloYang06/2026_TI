#ifndef HBALL_MISSION_CLIENT_H
#define HBALL_MISSION_CLIENT_H

#include "hball_mission_can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_MISSION_STATUS_FRESH_MS 150U

typedef struct
{
    uint16_t candidate_epoch;
    uint8_t selected_mission;
    uint8_t command;
    uint32_t command_time_ms;
    hball_mission_status_t latest_status;
    uint32_t last_status_ms;
    uint32_t accepted_status_total;
    uint32_t duplicate_status_total;
    uint32_t out_of_order_status_total;
    uint32_t epoch_mismatch_total;
    bool status_valid;
    bool start_requested;
} hball_mission_client_t;

void hball_mission_client_init(
    hball_mission_client_t *client, uint32_t now_ms
);
bool hball_mission_client_select(
    hball_mission_client_t *client,
    uint8_t mission_id,
    uint32_t now_ms
);
bool hball_mission_client_accept_status(
    hball_mission_client_t *client,
    const hball_mission_status_t *status,
    uint32_t now_ms
);
bool hball_mission_client_ready(
    const hball_mission_client_t *client, uint32_t now_ms
);
bool hball_mission_client_request_start(
    hball_mission_client_t *client, uint32_t now_ms
);
bool hball_mission_client_make_intent(
    const hball_mission_client_t *client,
    hball_mission_intent_t *intent
);

#ifdef __cplusplus
}
#endif

#endif

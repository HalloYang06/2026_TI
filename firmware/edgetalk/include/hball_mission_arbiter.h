#ifndef HBALL_MISSION_ARBITER_H
#define HBALL_MISSION_ARBITER_H

#include "hball_mission_can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_MISSION_READY_STABLE_MS 500U

typedef struct
{
    uint16_t epoch;
    uint8_t mission_id;
    uint8_t global_state;
    uint16_t ready_mask;
    uint16_t required_mask;
    uint8_t reason;
    uint8_t status_sequence;
    uint32_t prepare_time_ms;
    uint32_t ready_candidate_time_ms;
    uint32_t start_event_time_ms;
    uint32_t last_intent_time_ms;
    uint32_t prepare_accept_total;
    uint32_t start_accept_total;
    uint32_t start_reject_total;
    uint32_t epoch_reject_total;
    bool context_valid;
    bool ready_candidate_valid;
} hball_mission_arbiter_t;

void hball_mission_arbiter_init(hball_mission_arbiter_t *arbiter);
uint16_t hball_mission_required_ready_mask(uint8_t mission_id);
bool hball_mission_arbiter_accept_intent(
    hball_mission_arbiter_t *arbiter,
    const hball_mission_intent_t *intent,
    uint32_t now_ms
);
void hball_mission_arbiter_update_ready(
    hball_mission_arbiter_t *arbiter,
    uint16_t ready_mask,
    uint32_t now_ms
);
bool hball_mission_arbiter_mark_running(
    hball_mission_arbiter_t *arbiter
);
bool hball_mission_arbiter_mark_completed(
    hball_mission_arbiter_t *arbiter
);
bool hball_mission_arbiter_mark_aborted(
    hball_mission_arbiter_t *arbiter, uint8_t reason
);
bool hball_mission_arbiter_make_status(
    hball_mission_arbiter_t *arbiter,
    hball_mission_status_t *status
);

#ifdef __cplusplus
}
#endif

#endif

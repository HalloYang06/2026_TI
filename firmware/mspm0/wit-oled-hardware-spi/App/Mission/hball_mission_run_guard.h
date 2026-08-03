#ifndef HBALL_MISSION_RUN_GUARD_H
#define HBALL_MISSION_RUN_GUARD_H

#include "hball_mission_client.h"
#include "hball_mission_policy.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    HBALL_MISSION_RUN_CONTINUE = 0,
    HBALL_MISSION_RUN_STOP_REMOTE_COMPLETE,
    HBALL_MISSION_RUN_STOP_REMOTE_ABORT,
    HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE,
} hball_mission_run_decision_t;

hball_mission_run_decision_t hball_mission_run_guard_evaluate(
    const hball_mission_policy_t *policy,
    uint8_t expected_mission,
    uint16_t expected_epoch,
    const hball_mission_client_t *snapshot,
    uint32_t now_ms
);

#ifdef __cplusplus
}
#endif

#endif

#include "hball_mission_run_guard.h"

#include <stddef.h>

hball_mission_run_decision_t hball_mission_run_guard_evaluate(
    const hball_mission_policy_t *policy,
    uint8_t expected_mission,
    uint16_t expected_epoch,
    const hball_mission_client_t *snapshot,
    uint32_t now_ms
)
{
    uint8_t global_state;

    if (policy == NULL)
    {
        return HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE;
    }
    if ((policy->owner == HBALL_MISSION_OWNER_LOCAL_MSP)
        && policy->local_start
        && policy->chassis_allowed)
    {
        return HBALL_MISSION_RUN_CONTINUE;
    }
    if ((policy->owner != HBALL_MISSION_OWNER_REMOTE_M33)
        || !policy->wait_for_m33_running
        || !policy->chassis_allowed
        || (snapshot == NULL)
        || !snapshot->status_valid
        || (snapshot->selected_mission != expected_mission)
        || (snapshot->candidate_epoch != expected_epoch)
        || (snapshot->latest_status.mission_id != expected_mission)
        || (snapshot->latest_status.epoch != expected_epoch)
        || ((uint32_t)(now_ms - snapshot->last_status_ms)
            > HBALL_MISSION_STATUS_FRESH_MS))
    {
        return HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE;
    }

    global_state = snapshot->latest_status.global_state;
    if (global_state == HBALL_MISSION_STATE_RUNNING)
    {
        return HBALL_MISSION_RUN_CONTINUE;
    }
    if ((global_state == HBALL_MISSION_STATE_FINISHING)
        || (global_state == HBALL_MISSION_STATE_COMPLETED))
    {
        return HBALL_MISSION_RUN_STOP_REMOTE_COMPLETE;
    }
    if ((global_state == HBALL_MISSION_STATE_CONTROLLED_ABORT)
        || (global_state == HBALL_MISSION_STATE_FAULT_LATCHED))
    {
        return HBALL_MISSION_RUN_STOP_REMOTE_ABORT;
    }
    return HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE;
}

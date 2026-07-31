#include "hball_mission_policy.h"

#include <stddef.h>

static const hball_mission_policy_t g_invalid_policy = {
    HBALL_MISSION_OWNER_INVALID,
    false,
    false,
    false,
    false,
    false,
};

bool hball_mission_policy_get(
    uint8_t mission_id,
    hball_mission_policy_t *policy
)
{
    if (policy == NULL)
    {
        return false;
    }
    *policy = g_invalid_policy;

    switch (mission_id)
    {
    case HBALL_MISSION_Q2_FAST_LAP:
        policy->owner = HBALL_MISSION_OWNER_LOCAL_MSP;
        policy->local_start = true;
        policy->chassis_allowed = true;
        return true;

    case HBALL_MISSION_Q3_BALL_SEQUENCE:
        policy->owner = HBALL_MISSION_OWNER_REMOTE_M33;
        policy->wait_for_m33_running = true;
        policy->can_business_enabled = true;
        policy->imu_business_enabled = true;
        return true;

    case HBALL_MISSION_Q4_A_TO_B:
    case HBALL_MISSION_Q5_CENTER_LAP:
    case HBALL_MISSION_Q6_HOLD_POSITION_LAP:
        policy->owner = HBALL_MISSION_OWNER_REMOTE_M33;
        policy->chassis_allowed = true;
        policy->wait_for_m33_running = true;
        policy->can_business_enabled = true;
        policy->imu_business_enabled = true;
        return true;

    default:
        return false;
    }
}

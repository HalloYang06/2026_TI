#include "hball_mission_policy.h"

bool hball_mission_uses_can(uint8_t mission_id)
{
    return mission_id >= HBALL_MISSION_Q3_BALL_SEQUENCE
        && mission_id <= HBALL_MISSION_Q6_HOLD_POSITION_LAP;
}

bool hball_mission_runs_chassis(uint8_t mission_id)
{
    return mission_id == HBALL_MISSION_Q2_FAST_LAP
        || (mission_id >= HBALL_MISSION_Q4_A_TO_B
            && mission_id <= HBALL_MISSION_Q6_HOLD_POSITION_LAP);
}

bool hball_mission_runs_ball_control(uint8_t mission_id)
{
    return mission_id >= HBALL_MISSION_Q3_BALL_SEQUENCE
        && mission_id <= HBALL_MISSION_Q6_HOLD_POSITION_LAP;
}

#include "hball_mission_policy.h"

#include <assert.h>

int main(void)
{
    assert(!hball_mission_uses_can(HBALL_MISSION_Q2_FAST_LAP));
    assert(hball_mission_runs_chassis(HBALL_MISSION_Q2_FAST_LAP));
    assert(!hball_mission_runs_ball_control(HBALL_MISSION_Q2_FAST_LAP));

    assert(hball_mission_uses_can(HBALL_MISSION_Q3_BALL_SEQUENCE));
    assert(!hball_mission_runs_chassis(HBALL_MISSION_Q3_BALL_SEQUENCE));
    assert(hball_mission_runs_ball_control(HBALL_MISSION_Q3_BALL_SEQUENCE));

    assert(hball_mission_uses_can(HBALL_MISSION_Q4_A_TO_B));
    assert(hball_mission_runs_chassis(HBALL_MISSION_Q4_A_TO_B));
    assert(hball_mission_runs_ball_control(HBALL_MISSION_Q4_A_TO_B));

    assert(hball_mission_uses_can(HBALL_MISSION_Q5_CENTER_LAP));
    assert(hball_mission_runs_chassis(HBALL_MISSION_Q5_CENTER_LAP));
    assert(hball_mission_runs_ball_control(HBALL_MISSION_Q5_CENTER_LAP));

    assert(hball_mission_uses_can(HBALL_MISSION_Q6_HOLD_POSITION_LAP));
    assert(hball_mission_runs_chassis(HBALL_MISSION_Q6_HOLD_POSITION_LAP));
    assert(hball_mission_runs_ball_control(
        HBALL_MISSION_Q6_HOLD_POSITION_LAP));
    return 0;
}

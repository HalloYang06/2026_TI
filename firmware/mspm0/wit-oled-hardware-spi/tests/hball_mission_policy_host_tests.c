#include "hball_mission_policy.h"

#include <assert.h>

static void test_q2_is_local_and_does_not_depend_on_m33(void)
{
    hball_mission_policy_t policy;

    assert(hball_mission_policy_get(HBALL_MISSION_Q2_FAST_LAP, &policy));
    assert(policy.owner == HBALL_MISSION_OWNER_LOCAL_MSP);
    assert(policy.local_start);
    assert(policy.chassis_allowed);
    assert(!policy.wait_for_m33_running);
    assert(!policy.can_business_enabled);
    assert(!policy.imu_business_enabled);
}

static void test_q3_is_remote_and_keeps_the_chassis_inhibited(void)
{
    hball_mission_policy_t policy;

    assert(hball_mission_policy_get(HBALL_MISSION_Q3_BALL_SEQUENCE, &policy));
    assert(policy.owner == HBALL_MISSION_OWNER_REMOTE_M33);
    assert(!policy.local_start);
    assert(!policy.chassis_allowed);
    assert(policy.wait_for_m33_running);
    assert(policy.can_business_enabled);
    assert(policy.imu_business_enabled);
}

static void assert_distributed_chassis_mission(uint8_t mission_id)
{
    hball_mission_policy_t policy;

    assert(hball_mission_policy_get(mission_id, &policy));
    assert(policy.owner == HBALL_MISSION_OWNER_REMOTE_M33);
    assert(!policy.local_start);
    assert(policy.chassis_allowed);
    assert(policy.wait_for_m33_running);
    assert(policy.can_business_enabled);
    assert(policy.imu_business_enabled);
}

static void test_q4_through_q6_are_distributed_chassis_missions(void)
{
    assert_distributed_chassis_mission(HBALL_MISSION_Q4_A_TO_B);
    assert_distributed_chassis_mission(HBALL_MISSION_Q5_CENTER_LAP);
    assert_distributed_chassis_mission(HBALL_MISSION_Q6_HOLD_POSITION_LAP);
}

static void test_invalid_missions_fail_closed(void)
{
    hball_mission_policy_t policy = {
        HBALL_MISSION_OWNER_LOCAL_MSP,
        true,
        true,
        false,
        true,
        true,
    };

    assert(!hball_mission_policy_get(0U, &policy));
    assert(policy.owner == HBALL_MISSION_OWNER_INVALID);
    assert(!policy.local_start);
    assert(!policy.chassis_allowed);
    assert(!policy.wait_for_m33_running);
    assert(!policy.can_business_enabled);
    assert(!policy.imu_business_enabled);
    assert(!hball_mission_policy_get(HBALL_MISSION_Q2_FAST_LAP, NULL));
}

int main(void)
{
    test_q2_is_local_and_does_not_depend_on_m33();
    test_q3_is_remote_and_keeps_the_chassis_inhibited();
    test_q4_through_q6_are_distributed_chassis_missions();
    test_invalid_missions_fail_closed();
    return 0;
}

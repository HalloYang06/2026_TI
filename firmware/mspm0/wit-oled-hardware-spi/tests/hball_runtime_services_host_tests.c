#include "hball_mission_policy.h"
#include "hball_runtime_services.h"

#include <assert.h>

static void apply_mission(uint8_t mission_id)
{
    hball_mission_policy_t policy;

    assert(hball_mission_policy_get(mission_id, &policy));
    hball_runtime_services_apply_policy(&policy);
}

static void test_menu_keeps_links_available_for_task_selection(void)
{
    hball_runtime_services_enter_menu();
    assert(hball_runtime_services_can_enabled());
    assert(hball_runtime_services_imu_enabled());
}

static void test_q2_disables_competition_can_and_imu_business_work(void)
{
    hball_runtime_services_enter_menu();
    apply_mission(HBALL_MISSION_Q2_FAST_LAP);
    assert(!hball_runtime_services_can_enabled());
    assert(!hball_runtime_services_imu_enabled());
}

static void test_q3_through_q6_keep_distributed_services_enabled(void)
{
    uint8_t mission_id;

    for (mission_id = HBALL_MISSION_Q3_BALL_SEQUENCE;
         mission_id <= HBALL_MISSION_Q6_HOLD_POSITION_LAP;
         ++mission_id)
    {
        apply_mission(mission_id);
        assert(hball_runtime_services_can_enabled());
        assert(hball_runtime_services_imu_enabled());
    }
}

static void test_missing_policy_fails_closed(void)
{
    hball_runtime_services_enter_menu();
    hball_runtime_services_apply_policy(NULL);
    assert(!hball_runtime_services_can_enabled());
    assert(!hball_runtime_services_imu_enabled());
}

int main(void)
{
    test_menu_keeps_links_available_for_task_selection();
    test_q2_disables_competition_can_and_imu_business_work();
    test_q3_through_q6_keep_distributed_services_enabled();
    test_missing_policy_fails_closed();
    return 0;
}

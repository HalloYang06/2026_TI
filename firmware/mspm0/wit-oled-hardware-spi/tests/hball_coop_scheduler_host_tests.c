#include "hball_coop_scheduler.h"

#include <assert.h>

static void test_period_table_matches_control_architecture(void)
{
    assert(hball_coop_scheduler_period_ms(HBALL_COOP_TASK_SAFETY) == 1U);
    assert(hball_coop_scheduler_period_ms(HBALL_COOP_TASK_CAN) == 1U);
    assert(hball_coop_scheduler_period_ms(HBALL_COOP_TASK_IMU) == 1U);
    assert(hball_coop_scheduler_period_ms(HBALL_COOP_TASK_MISSION) == 5U);
    assert(hball_coop_scheduler_period_ms(HBALL_COOP_TASK_BUTTON) == 5U);
    assert(hball_coop_scheduler_period_ms(HBALL_COOP_TASK_LINE) == 10U);
    assert(hball_coop_scheduler_period_ms(HBALL_COOP_TASK_WHEEL) == 100U);
    assert(hball_coop_scheduler_period_ms(HBALL_COOP_TASK_LCD) == 100U);
    assert(hball_coop_scheduler_period_ms(HBALL_COOP_TASK_COUNT) == 0U);
}

static void test_tick_publishes_each_task_at_its_period(void)
{
    hball_coop_scheduler_t scheduler;
    uint32_t tick;

    hball_coop_scheduler_init(&scheduler);
    for (tick = 1U; tick <= 4U; ++tick)
    {
        hball_coop_scheduler_tick_isr(&scheduler);
    }
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_SAFETY) == 3U);
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_CAN) == 3U);
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_IMU) == 3U);
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_MISSION) == 0U);

    hball_coop_scheduler_tick_isr(&scheduler);
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_MISSION) == 1U);
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_BUTTON) == 1U);

    for (tick = 6U; tick <= 10U; ++tick)
    {
        hball_coop_scheduler_tick_isr(&scheduler);
    }
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_LINE) == 1U);
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_WHEEL) == 0U);

    for (tick = 11U; tick <= 100U; ++tick)
    {
        hball_coop_scheduler_tick_isr(&scheduler);
    }
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_WHEEL) == 1U);
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_LCD) == 1U);
}

static void test_pending_work_saturates_and_records_deadline_misses(void)
{
    hball_coop_scheduler_t scheduler;
    uint32_t tick;

    hball_coop_scheduler_init(&scheduler);
    for (tick = 0U; tick < 10U; ++tick)
    {
        hball_coop_scheduler_tick_isr(&scheduler);
    }

    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_SAFETY)
           == HBALL_COOP_MAX_PENDING);
    assert(hball_coop_scheduler_missed(&scheduler, HBALL_COOP_TASK_SAFETY)
           == 9U);
    assert(hball_coop_scheduler_take(&scheduler, HBALL_COOP_TASK_SAFETY));
    assert(hball_coop_scheduler_pending(&scheduler, HBALL_COOP_TASK_SAFETY)
           == HBALL_COOP_MAX_PENDING - 1U);
}

static void test_invalid_access_fails_closed(void)
{
    hball_coop_scheduler_t scheduler;

    hball_coop_scheduler_init(&scheduler);
    assert(!hball_coop_scheduler_take(NULL, HBALL_COOP_TASK_CAN));
    assert(!hball_coop_scheduler_take(&scheduler, HBALL_COOP_TASK_COUNT));
    assert(hball_coop_scheduler_pending(NULL, HBALL_COOP_TASK_CAN) == 0U);
    assert(hball_coop_scheduler_missed(&scheduler, HBALL_COOP_TASK_COUNT) == 0U);
}

int main(void)
{
    test_period_table_matches_control_architecture();
    test_tick_publishes_each_task_at_its_period();
    test_pending_work_saturates_and_records_deadline_misses();
    test_invalid_access_fails_closed();
    return 0;
}

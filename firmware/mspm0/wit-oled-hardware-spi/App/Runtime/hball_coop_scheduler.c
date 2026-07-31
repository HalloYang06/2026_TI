#include "hball_coop_scheduler.h"

#include <stddef.h>

static const uint16_t g_hball_coop_period_ms[HBALL_COOP_TASK_COUNT] = {
    1U,
    1U,
    5U,
    5U,
    10U,
    100U,
    100U,
};

static bool hball_coop_task_valid(hball_coop_task_t task)
{
    return (uint32_t)task < (uint32_t)HBALL_COOP_TASK_COUNT;
}

void hball_coop_scheduler_init(hball_coop_scheduler_t *scheduler)
{
    uint8_t task;

    if (scheduler == NULL)
    {
        return;
    }
    for (task = 0U; task < (uint8_t)HBALL_COOP_TASK_COUNT; ++task)
    {
        scheduler->countdown_ms[task] = g_hball_coop_period_ms[task];
        scheduler->pending[task] = 0U;
        scheduler->missed[task] = 0U;
    }
}

void hball_coop_scheduler_tick_isr(hball_coop_scheduler_t *scheduler)
{
    uint8_t task;

    if (scheduler == NULL)
    {
        return;
    }
    for (task = 0U; task < (uint8_t)HBALL_COOP_TASK_COUNT; ++task)
    {
        if (scheduler->countdown_ms[task] > 1U)
        {
            scheduler->countdown_ms[task]--;
            continue;
        }

        scheduler->countdown_ms[task] = g_hball_coop_period_ms[task];
        if (scheduler->pending[task] != 0U)
        {
            scheduler->missed[task]++;
        }
        if (scheduler->pending[task] < HBALL_COOP_MAX_PENDING)
        {
            scheduler->pending[task]++;
        }
    }
}

bool hball_coop_scheduler_take(
    hball_coop_scheduler_t *scheduler,
    hball_coop_task_t task
)
{
    if ((scheduler == NULL)
        || !hball_coop_task_valid(task)
        || (scheduler->pending[task] == 0U))
    {
        return false;
    }
    scheduler->pending[task]--;
    return true;
}

uint8_t hball_coop_scheduler_pending(
    const hball_coop_scheduler_t *scheduler,
    hball_coop_task_t task
)
{
    if ((scheduler == NULL) || !hball_coop_task_valid(task))
    {
        return 0U;
    }
    return scheduler->pending[task];
}

uint32_t hball_coop_scheduler_missed(
    const hball_coop_scheduler_t *scheduler,
    hball_coop_task_t task
)
{
    if ((scheduler == NULL) || !hball_coop_task_valid(task))
    {
        return 0U;
    }
    return scheduler->missed[task];
}

uint16_t hball_coop_scheduler_period_ms(hball_coop_task_t task)
{
    if (!hball_coop_task_valid(task))
    {
        return 0U;
    }
    return g_hball_coop_period_ms[task];
}

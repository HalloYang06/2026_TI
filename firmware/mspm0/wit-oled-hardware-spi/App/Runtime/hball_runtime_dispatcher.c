#include "hball_runtime_dispatcher.h"

#include <stddef.h>

static bool hball_runtime_hooks_valid(
    const hball_runtime_dispatcher_hooks_t *hooks
)
{
    return (hooks != NULL)
        && (hooks->enter_critical != NULL)
        && (hooks->exit_critical != NULL)
        && (hooks->can_enabled != NULL)
        && (hooks->can_service != NULL);
}

bool hball_runtime_dispatcher_init(
    hball_runtime_dispatcher_t *dispatcher,
    const hball_runtime_dispatcher_hooks_t *hooks
)
{
    if (dispatcher == NULL)
    {
        return false;
    }
    dispatcher->initialized = false;
    if (!hball_runtime_hooks_valid(hooks))
    {
        return false;
    }

    dispatcher->hooks = *hooks;
    hball_coop_scheduler_init(&dispatcher->scheduler);
    dispatcher->initialized = true;
    return true;
}

void hball_runtime_dispatcher_tick_isr(
    hball_runtime_dispatcher_t *dispatcher
)
{
    if ((dispatcher == NULL) || !dispatcher->initialized)
    {
        return;
    }
    hball_coop_scheduler_tick_isr(&dispatcher->scheduler);
}

void hball_runtime_dispatcher_poll(
    hball_runtime_dispatcher_t *dispatcher,
    uint32_t now_ms
)
{
    uint8_t consumed = 0U;

    if ((dispatcher == NULL) || !dispatcher->initialized)
    {
        return;
    }

    dispatcher->hooks.enter_critical(dispatcher->hooks.context);
    while ((consumed < HBALL_COOP_MAX_PENDING)
        && hball_coop_scheduler_take(
            &dispatcher->scheduler, HBALL_COOP_TASK_CAN))
    {
        consumed++;
    }
    dispatcher->hooks.exit_critical(dispatcher->hooks.context);

    if ((consumed != 0U)
        && dispatcher->hooks.can_enabled(dispatcher->hooks.context))
    {
        dispatcher->hooks.can_service(dispatcher->hooks.context, now_ms);
    }
}

uint8_t hball_runtime_dispatcher_pending(
    const hball_runtime_dispatcher_t *dispatcher,
    hball_coop_task_t task
)
{
    if ((dispatcher == NULL) || !dispatcher->initialized)
    {
        return 0U;
    }
    return hball_coop_scheduler_pending(&dispatcher->scheduler, task);
}

uint32_t hball_runtime_dispatcher_missed(
    const hball_runtime_dispatcher_t *dispatcher,
    hball_coop_task_t task
)
{
    if ((dispatcher == NULL) || !dispatcher->initialized)
    {
        return 0U;
    }
    return hball_coop_scheduler_missed(&dispatcher->scheduler, task);
}

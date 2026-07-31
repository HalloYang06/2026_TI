#include "hball_runtime_target.h"

#include "hball_can_port.h"
#include "hball_runtime_dispatcher.h"
#include "hball_runtime_services.h"
#include "ti_msp_dl_config.h"

#include <string.h>

typedef struct
{
    uint32_t interrupt_state;
} hball_runtime_target_context_t;

static hball_runtime_dispatcher_t g_hball_runtime_dispatcher;
static hball_runtime_target_context_t g_hball_runtime_context;
volatile hball_runtime_target_stats_t g_hball_runtime_target_stats;

static void hball_runtime_target_enter_critical(void *context)
{
    hball_runtime_target_context_t *target =
        (hball_runtime_target_context_t *)context;

    target->interrupt_state = __get_PRIMASK();
    __disable_irq();
}

static void hball_runtime_target_exit_critical(void *context)
{
    const hball_runtime_target_context_t *target =
        (const hball_runtime_target_context_t *)context;

    if (target->interrupt_state == 0U)
    {
        __enable_irq();
    }
}

static bool hball_runtime_target_can_enabled(void *context)
{
    (void)context;
    return hball_runtime_services_can_enabled();
}

static void hball_runtime_target_can_service(
    void *context,
    uint32_t now_ms
)
{
    (void)context;
    hball_can_port_tick_1ms(now_ms);
    g_hball_runtime_target_stats.can_service_total++;
    g_hball_runtime_target_stats.last_can_service_ms = now_ms;
}

bool hball_runtime_target_init(void)
{
    const hball_runtime_dispatcher_hooks_t hooks = {
        hball_runtime_target_enter_critical,
        hball_runtime_target_exit_critical,
        hball_runtime_target_can_enabled,
        hball_runtime_target_can_service,
        &g_hball_runtime_context,
    };
    bool initialized;

    memset(&g_hball_runtime_context, 0, sizeof(g_hball_runtime_context));
    memset((void *)&g_hball_runtime_target_stats, 0,
           sizeof(g_hball_runtime_target_stats));
    initialized = hball_runtime_dispatcher_init(
        &g_hball_runtime_dispatcher, &hooks
    );
    g_hball_runtime_target_stats.initialized = initialized ? 1U : 0U;
    return initialized;
}

void hball_runtime_target_tick_isr(void)
{
    if (g_hball_runtime_target_stats.initialized == 0U)
    {
        return;
    }
    hball_runtime_dispatcher_tick_isr(&g_hball_runtime_dispatcher);
    g_hball_runtime_target_stats.tick_total++;
}

void hball_runtime_target_poll(uint32_t now_ms)
{
    if (g_hball_runtime_target_stats.initialized == 0U)
    {
        return;
    }
    hball_runtime_dispatcher_poll(&g_hball_runtime_dispatcher, now_ms);
    g_hball_runtime_target_stats.poll_total++;
    g_hball_runtime_target_stats.last_poll_ms = now_ms;
    g_hball_runtime_target_stats.can_pending =
        hball_runtime_dispatcher_pending(
            &g_hball_runtime_dispatcher, HBALL_COOP_TASK_CAN
        );
    g_hball_runtime_target_stats.can_deadline_miss_total =
        hball_runtime_dispatcher_missed(
            &g_hball_runtime_dispatcher, HBALL_COOP_TASK_CAN
        );
}

uint32_t hball_runtime_target_missed(hball_coop_task_t task)
{
    return hball_runtime_dispatcher_missed(
        &g_hball_runtime_dispatcher, task
    );
}

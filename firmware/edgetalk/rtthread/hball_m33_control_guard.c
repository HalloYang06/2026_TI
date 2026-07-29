#include "hball_control_guard.h"
#include "hball_dualcore_platform.h"
#include "hball_m33_inputs.h"

#include <finsh.h>
#include <rtthread.h>

#define HBALL_M33_GUARD_PERIOD_MS 1U
#define HBALL_M33_GUARD_LOG_PERIOD_MS 1000U

_Static_assert(
    HBALL_CONTROL_GUARD_ACTUATOR_TX_ENABLED == 0U,
    "H-ball M33 actuator transmission must remain disabled"
);

static hball_control_guard_t g_hball_m33_guard;
static hball_control_decision_t g_hball_m33_decision;
static hball_control_shadow_t g_hball_m33_shadow;
static rt_thread_t g_hball_m33_guard_worker = RT_NULL;
static rt_uint32_t g_hball_m33_ipc_read_total = 0U;
static rt_uint32_t g_hball_m33_ipc_busy_total = 0U;
static rt_uint32_t g_hball_m33_ipc_failure_total = 0U;
static rt_uint32_t g_hball_m33_snapshot_failure_total = 0U;
static rt_uint32_t g_hball_m33_stale_event_total = 0U;
static hball_ipc_result_t g_hball_m33_last_ipc_result = HBALL_IPC_HEADER;
static rt_bool_t g_hball_m33_fresh_previous = RT_FALSE;

static void hball_m33_guard_print_status(void)
{
    const rt_uint32_t now_ms = (rt_uint32_t)rt_tick_get_millisecond();
    const rt_bool_t fresh = hball_control_guard_is_fresh(
        &g_hball_m33_guard, now_ms
    ) ? RT_TRUE : RT_FALSE;

    rt_kprintf(
        "[hball-guard] rate_hz=1000 ipc=%lu/%lu/%lu/%d snap_fail=%lu qualified=%lu rejected=%lu duplicate=%lu stale=%lu fresh=%d step=%lu source=%lu reason=%u target_urad=%ld ACTUATOR_TX=0\n",
        (unsigned long)g_hball_m33_ipc_read_total,
        (unsigned long)g_hball_m33_ipc_busy_total,
        (unsigned long)g_hball_m33_ipc_failure_total,
        (int)g_hball_m33_last_ipc_result,
        (unsigned long)g_hball_m33_snapshot_failure_total,
        (unsigned long)g_hball_m33_guard.qualified_total,
        (unsigned long)g_hball_m33_guard.rejected_total,
        (unsigned long)g_hball_m33_guard.duplicate_total,
        (unsigned long)g_hball_m33_stale_event_total,
        (int)fresh,
        (unsigned long)g_hball_m33_shadow.controller_steps,
        (unsigned long)g_hball_m33_shadow.source_sensor_sequence,
        (unsigned)g_hball_m33_guard.last_reason,
        (long)(g_hball_m33_decision.shadow_target_rad * 1000000.0F)
    );
}

static void hball_m33_guard_step(rt_uint32_t now_ms)
{
    hball_control_shadow_t shadow;

    g_hball_m33_last_ipc_result = hball_ipc_control_read(
        &hball_ipc_platform_region()->control,
        &shadow,
        hball_ipc_platform_cache_ops()
    );
    if (g_hball_m33_last_ipc_result == HBALL_IPC_OK)
    {
        hball_sensor_snapshot_t sensor;

        g_hball_m33_ipc_read_total++;
        g_hball_m33_shadow = shadow;
        if (hball_m33_inputs_get_snapshot(&sensor))
        {
            (void)hball_control_guard_observe(
                &g_hball_m33_guard,
                &shadow,
                &sensor,
                now_ms,
                &g_hball_m33_decision
            );
        }
        else
        {
            g_hball_m33_snapshot_failure_total++;
        }
    }
    else if (g_hball_m33_last_ipc_result == HBALL_IPC_BUSY)
    {
        g_hball_m33_ipc_busy_total++;
    }
    else
    {
        g_hball_m33_ipc_failure_total++;
    }
    {
        const rt_bool_t fresh = hball_control_guard_is_fresh(
            &g_hball_m33_guard, now_ms
        ) ? RT_TRUE : RT_FALSE;

        if (g_hball_m33_fresh_previous && !fresh)
        {
            g_hball_m33_stale_event_total++;
        }
        g_hball_m33_fresh_previous = fresh;
    }
}

static void hball_m33_guard_worker_entry(void *parameter)
{
    rt_tick_t release_tick = rt_tick_get();
    const rt_tick_t period_ticks =
        rt_tick_from_millisecond(HBALL_M33_GUARD_PERIOD_MS);
    rt_uint32_t last_log_ms = (rt_uint32_t)rt_tick_get_millisecond();

    RT_UNUSED(parameter);
    while (1)
    {
        const rt_uint32_t now_ms =
            (rt_uint32_t)rt_tick_get_millisecond();

        hball_m33_guard_step(now_ms);
        if ((rt_uint32_t)(now_ms - last_log_ms)
            >= HBALL_M33_GUARD_LOG_PERIOD_MS)
        {
            last_log_ms = now_ms;
            hball_m33_guard_print_status();
        }
        (void)rt_thread_delay_until(&release_tick, period_ticks);
    }
}

static void hball_guard_status(void)
{
    hball_m33_guard_print_status();
}
MSH_CMD_EXPORT(hball_guard_status, show M33 shadow-only safety diagnostics);

static int hball_m33_control_guard_init(void)
{
    hball_control_guard_init(&g_hball_m33_guard);
    rt_memset(&g_hball_m33_decision, 0, sizeof(g_hball_m33_decision));
    rt_memset(&g_hball_m33_shadow, 0, sizeof(g_hball_m33_shadow));
    g_hball_m33_guard_worker = rt_thread_create(
        "hball_safe",
        hball_m33_guard_worker_entry,
        RT_NULL,
        3072U,
        13U,
        5U
    );
    if (g_hball_m33_guard_worker == RT_NULL)
    {
        return -RT_ERROR;
    }
    rt_thread_startup(g_hball_m33_guard_worker);
    rt_kprintf(
        "[hball-guard] M33 1 kHz shadow observer; hard-disabled ACTUATOR_TX=0\n"
    );
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(hball_m33_control_guard_init);

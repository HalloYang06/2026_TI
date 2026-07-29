#include "hball_m33_inputs.h"
#include "hball_dualcore_platform.h"

#include <finsh.h>
#include <rtthread.h>

#define HBALL_M33_SNAPSHOT_PERIOD_MS 5U
#define HBALL_M33_INPUT_LOG_PERIOD_MS 1000U

static hball_sensor_fusion_t g_hball_sensor_fusion;
static hball_sensor_snapshot_t g_hball_latest_snapshot;
static rt_mutex_t g_hball_input_mutex = RT_NULL;
static rt_thread_t g_hball_snapshot_worker = RT_NULL;
static rt_uint32_t g_hball_snapshot_total = 0U;
static rt_uint32_t g_hball_lock_failure_total = 0U;
static rt_uint32_t g_hball_ipc_publish_total = 0U;
static rt_uint32_t g_hball_ipc_publish_failure_total = 0U;
static hball_ipc_result_t g_hball_ipc_last_result = HBALL_IPC_HEADER;

static rt_bool_t hball_m33_inputs_lock(void)
{
    if ((g_hball_input_mutex == RT_NULL)
        || (rt_mutex_take(g_hball_input_mutex, RT_WAITING_FOREVER) != RT_EOK))
    {
        g_hball_lock_failure_total++;
        return RT_FALSE;
    }
    return RT_TRUE;
}

static void hball_m33_inputs_unlock(void)
{
    (void)rt_mutex_release(g_hball_input_mutex);
}

bool hball_m33_inputs_publish_vision(
    const hball_vision_measurement_t *measurement, uint32_t receive_ms
)
{
    if ((measurement == RT_NULL) || !hball_m33_inputs_lock())
    {
        return false;
    }
    hball_sensor_fusion_set_vision(
        &g_hball_sensor_fusion, measurement, receive_ms
    );
    hball_m33_inputs_unlock();
    return true;
}

bool hball_m33_inputs_publish_msp(const hball_msp_monitor_t *monitor)
{
    if ((monitor == RT_NULL) || !hball_m33_inputs_lock())
    {
        return false;
    }
    hball_sensor_fusion_set_msp(&g_hball_sensor_fusion, monitor);
    hball_m33_inputs_unlock();
    return true;
}

bool hball_m33_inputs_publish_motor(
    const hball_motor_feedback_t *feedback, uint32_t receive_ms
)
{
    if ((feedback == RT_NULL) || !hball_m33_inputs_lock())
    {
        return false;
    }
    hball_sensor_fusion_set_motor(
        &g_hball_sensor_fusion, feedback, receive_ms
    );
    hball_m33_inputs_unlock();
    return true;
}

bool hball_m33_inputs_get_snapshot(hball_sensor_snapshot_t *snapshot)
{
    if ((snapshot == RT_NULL) || !hball_m33_inputs_lock())
    {
        return false;
    }
    *snapshot = g_hball_latest_snapshot;
    hball_m33_inputs_unlock();
    return true;
}

static void hball_m33_inputs_print_status(void)
{
    hball_sensor_snapshot_t snapshot;

    if (!hball_m33_inputs_get_snapshot(&snapshot))
    {
        rt_kprintf("[hball-inputs] snapshot unavailable lock_fail=%lu ACTUATOR_TX=0\n",
            (unsigned long)g_hball_lock_failure_total);
        return;
    }
    rt_kprintf(
        "[hball-inputs] snapshots=%lu ipc=%lu/%lu/%d seq=%lu valid=0x%08lx vision_seq=%lu vision_age=%lu imu_age=%lu motor_age=%lu ACTUATOR_TX=0\n",
        (unsigned long)g_hball_snapshot_total,
        (unsigned long)g_hball_ipc_publish_total,
        (unsigned long)g_hball_ipc_publish_failure_total,
        (int)g_hball_ipc_last_result,
        (unsigned long)snapshot.sequence,
        (unsigned long)snapshot.valid_flags,
        (unsigned long)snapshot.vision_sequence,
        (unsigned long)snapshot.vision_receive_age_ms,
        (unsigned long)snapshot.imu_age_ms,
        (unsigned long)snapshot.motor_age_ms
    );
}

static void hball_m33_snapshot_worker_entry(void *parameter)
{
    rt_uint32_t last_log_ms = (rt_uint32_t)rt_tick_get_millisecond();

    RT_UNUSED(parameter);
    while (1)
    {
        const rt_uint32_t now_ms = (rt_uint32_t)rt_tick_get_millisecond();
        hball_sensor_snapshot_t snapshot;
        rt_bool_t snapshot_ready = RT_FALSE;

        if (hball_m33_inputs_lock())
        {
            hball_sensor_fusion_snapshot(
                &g_hball_sensor_fusion, now_ms, &snapshot
            );
            g_hball_latest_snapshot = snapshot;
            g_hball_snapshot_total++;
            snapshot_ready = RT_TRUE;
            hball_m33_inputs_unlock();
        }
        if (snapshot_ready)
        {
            g_hball_ipc_last_result = hball_ipc_sensor_publish(
                &hball_ipc_platform_region()->sensor,
                &snapshot,
                hball_ipc_platform_cache_ops()
            );
            if (g_hball_ipc_last_result == HBALL_IPC_OK)
            {
                g_hball_ipc_publish_total++;
            }
            else
            {
                g_hball_ipc_publish_failure_total++;
            }
        }
        if ((rt_uint32_t)(now_ms - last_log_ms) >= HBALL_M33_INPUT_LOG_PERIOD_MS)
        {
            last_log_ms = now_ms;
            hball_m33_inputs_print_status();
        }
        rt_thread_mdelay(HBALL_M33_SNAPSHOT_PERIOD_MS);
    }
}

static void hball_inputs_status(void)
{
    hball_m33_inputs_print_status();
}
MSH_CMD_EXPORT(hball_inputs_status, show combined read-only sensor snapshot);

static int hball_m33_inputs_init(void)
{
    hball_sensor_fusion_init(&g_hball_sensor_fusion);
    rt_memset(&g_hball_latest_snapshot, 0, sizeof(g_hball_latest_snapshot));
    hball_ipc_region_reset(
        hball_ipc_platform_region(), hball_ipc_platform_cache_ops()
    );
    g_hball_input_mutex = rt_mutex_create("hball_in", RT_IPC_FLAG_PRIO);
    if (g_hball_input_mutex == RT_NULL)
    {
        return -RT_ERROR;
    }
    g_hball_snapshot_worker = rt_thread_create(
        "hball_snap",
        hball_m33_snapshot_worker_entry,
        RT_NULL,
        3072U,
        14U,
        10U
    );
    if (g_hball_snapshot_worker == RT_NULL)
    {
        return -RT_ERROR;
    }
    rt_thread_startup(g_hball_snapshot_worker);
    rt_kprintf(
        "[hball-inputs] M33 200 Hz USB+CAN snapshot; shadow only ACTUATOR_TX=0\n"
    );
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(hball_m33_inputs_init);

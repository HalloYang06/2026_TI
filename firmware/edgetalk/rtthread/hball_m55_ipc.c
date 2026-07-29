#include "hball_m55_ipc.h"

#include "hball_dualcore_platform.h"

#include <stddef.h>

static hball_m55_ipc_diag_t g_hball_m55_ipc_diag = {
    .last_sensor_result = HBALL_IPC_HEADER,
    .last_control_result = HBALL_IPC_HEADER,
};

bool hball_m55_read_sensor_snapshot(hball_sensor_snapshot_t *snapshot)
{
    hball_ipc_result_t result;

    if (snapshot == NULL)
    {
        return false;
    }
    result = hball_ipc_sensor_read(
        &hball_ipc_platform_region()->sensor,
        snapshot,
        hball_ipc_platform_cache_ops()
    );
    g_hball_m55_ipc_diag.last_sensor_result = result;
    if (result == HBALL_IPC_OK)
    {
        g_hball_m55_ipc_diag.sensor_read_total++;
        return true;
    }
    if (result == HBALL_IPC_BUSY)
    {
        g_hball_m55_ipc_diag.sensor_busy_total++;
    }
    else
    {
        g_hball_m55_ipc_diag.sensor_failure_total++;
    }
    return false;
}

bool hball_m55_publish_control_shadow(
    const hball_control_shadow_t *shadow
)
{
    hball_ipc_result_t result;

    if (shadow == NULL)
    {
        return false;
    }
    result = hball_ipc_control_publish(
        &hball_ipc_platform_region()->control,
        shadow,
        hball_ipc_platform_cache_ops()
    );
    g_hball_m55_ipc_diag.last_control_result = result;
    if (result == HBALL_IPC_OK)
    {
        g_hball_m55_ipc_diag.control_publish_total++;
        return true;
    }
    g_hball_m55_ipc_diag.control_failure_total++;
    return false;
}

void hball_m55_ipc_get_diag(hball_m55_ipc_diag_t *diagnostic)
{
    if (diagnostic != NULL)
    {
        *diagnostic = g_hball_m55_ipc_diag;
    }
}

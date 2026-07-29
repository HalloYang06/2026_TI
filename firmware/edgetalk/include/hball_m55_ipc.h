#ifndef HBALL_M55_IPC_H
#define HBALL_M55_IPC_H

#include "hball_dualcore_ipc.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t sensor_read_total;
    uint32_t sensor_busy_total;
    uint32_t sensor_failure_total;
    uint32_t control_publish_total;
    uint32_t control_failure_total;
    hball_ipc_result_t last_sensor_result;
    hball_ipc_result_t last_control_result;
} hball_m55_ipc_diag_t;

bool hball_m55_read_sensor_snapshot(hball_sensor_snapshot_t *snapshot);
bool hball_m55_publish_control_shadow(
    const hball_control_shadow_t *shadow
);
void hball_m55_ipc_get_diag(hball_m55_ipc_diag_t *diagnostic);

#ifdef __cplusplus
}
#endif

#endif

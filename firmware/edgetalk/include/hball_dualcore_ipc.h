#ifndef HBALL_DUALCORE_IPC_H
#define HBALL_DUALCORE_IPC_H

#include "hball_sensor_fusion.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#define HBALL_IPC_ALIGNAS(value) alignas(value)
#else
#define HBALL_IPC_ALIGNAS(value) _Alignas(value)
#endif

#define HBALL_IPC_MAGIC UINT32_C(0x50494248)
#define HBALL_IPC_VERSION 1U
#define HBALL_IPC_SENSOR_MESSAGE_TYPE 1U
#define HBALL_IPC_CONTROL_MESSAGE_TYPE 2U
#define HBALL_IPC_CACHE_LINE_SIZE 32U
#define HBALL_IPC_HEADER_SIZE 32U
#define HBALL_IPC_SENSOR_PAYLOAD_SIZE 96U
#define HBALL_IPC_CONTROL_PAYLOAD_SIZE 32U
#define HBALL_IPC_SENSOR_FRAME_SIZE 160U
#define HBALL_IPC_CONTROL_FRAME_SIZE 96U
#define HBALL_IPC_SHARED_REGION_SIZE 256U

#define HBALL_IPC_CONTROL_FLAG_SAFETY_ELIGIBLE (UINT16_C(1) << 0)
#define HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY (UINT16_C(1) << 1)
#define HBALL_IPC_CONTROL_MODE_MAX 4U

typedef enum
{
    HBALL_IPC_OK = 0,
    HBALL_IPC_ARGUMENT,
    HBALL_IPC_BUSY,
    HBALL_IPC_HEADER,
    HBALL_IPC_CRC,
    HBALL_IPC_RANGE,
} hball_ipc_result_t;

typedef void (*hball_ipc_cache_operation_t)(
    void *address, size_t length, void *context
);
typedef void (*hball_ipc_barrier_t)(void *context);

typedef struct
{
    hball_ipc_cache_operation_t clean;
    hball_ipc_cache_operation_t invalidate;
    hball_ipc_barrier_t barrier;
    void *context;
} hball_ipc_cache_ops_t;

typedef struct
{
    HBALL_IPC_ALIGNAS(HBALL_IPC_CACHE_LINE_SIZE)
        uint8_t bytes[HBALL_IPC_SENSOR_FRAME_SIZE];
} hball_ipc_sensor_slot_t;

typedef struct
{
    HBALL_IPC_ALIGNAS(HBALL_IPC_CACHE_LINE_SIZE)
        uint8_t bytes[HBALL_IPC_CONTROL_FRAME_SIZE];
} hball_ipc_control_slot_t;

typedef struct
{
    hball_ipc_sensor_slot_t sensor;
    hball_ipc_control_slot_t control;
} hball_ipc_shared_region_t;

typedef struct
{
    uint32_t source_sensor_sequence;
    uint32_t controller_steps;
    uint32_t deadline_misses;
    uint32_t produced_time_ms;
    uint16_t mode;
    uint16_t flags;
    float target_angle_rad;
    float estimated_position_m;
    float estimated_velocity_mps;
    float estimated_disturbance_mps2;
} hball_control_shadow_t;

uint32_t hball_ipc_crc32c(const uint8_t *data, size_t length);
void hball_ipc_region_reset(
    hball_ipc_shared_region_t *region, const hball_ipc_cache_ops_t *cache_ops
);
hball_ipc_result_t hball_ipc_sensor_publish(
    hball_ipc_sensor_slot_t *slot,
    const hball_sensor_snapshot_t *snapshot,
    const hball_ipc_cache_ops_t *cache_ops
);
hball_ipc_result_t hball_ipc_sensor_read(
    const hball_ipc_sensor_slot_t *slot,
    hball_sensor_snapshot_t *snapshot,
    const hball_ipc_cache_ops_t *cache_ops
);
hball_ipc_result_t hball_ipc_control_publish(
    hball_ipc_control_slot_t *slot,
    const hball_control_shadow_t *shadow,
    const hball_ipc_cache_ops_t *cache_ops
);
hball_ipc_result_t hball_ipc_control_read(
    const hball_ipc_control_slot_t *slot,
    hball_control_shadow_t *shadow,
    const hball_ipc_cache_ops_t *cache_ops
);

#ifdef __cplusplus
}
#endif

#undef HBALL_IPC_ALIGNAS

#endif

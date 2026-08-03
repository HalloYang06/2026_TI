#ifndef HBALL_RUNTIME_DISPATCHER_H
#define HBALL_RUNTIME_DISPATCHER_H

#include "hball_coop_scheduler.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*hball_runtime_critical_fn)(void *context);
typedef bool (*hball_runtime_enabled_fn)(void *context);
typedef void (*hball_runtime_service_fn)(void *context, uint32_t now_ms);

typedef struct
{
    hball_runtime_critical_fn enter_critical;
    hball_runtime_critical_fn exit_critical;
    hball_runtime_enabled_fn can_enabled;
    hball_runtime_service_fn can_service;
    hball_runtime_enabled_fn imu_enabled;
    hball_runtime_service_fn imu_service;
    void *context;
} hball_runtime_dispatcher_hooks_t;

typedef struct
{
    hball_coop_scheduler_t scheduler;
    hball_runtime_dispatcher_hooks_t hooks;
    bool initialized;
} hball_runtime_dispatcher_t;

bool hball_runtime_dispatcher_init(
    hball_runtime_dispatcher_t *dispatcher,
    const hball_runtime_dispatcher_hooks_t *hooks
);
void hball_runtime_dispatcher_tick_isr(
    hball_runtime_dispatcher_t *dispatcher
);
void hball_runtime_dispatcher_poll(
    hball_runtime_dispatcher_t *dispatcher,
    uint32_t now_ms
);
uint8_t hball_runtime_dispatcher_pending(
    const hball_runtime_dispatcher_t *dispatcher,
    hball_coop_task_t task
);
uint32_t hball_runtime_dispatcher_missed(
    const hball_runtime_dispatcher_t *dispatcher,
    hball_coop_task_t task
);

#ifdef __cplusplus
}
#endif

#endif

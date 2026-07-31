#ifndef HBALL_RUNTIME_TARGET_H
#define HBALL_RUNTIME_TARGET_H

#include "hball_coop_scheduler.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t tick_total;
    uint32_t poll_total;
    uint32_t can_service_total;
    uint32_t imu_service_total;
    uint32_t can_deadline_miss_total;
    uint32_t imu_deadline_miss_total;
    uint32_t last_poll_ms;
    uint32_t last_can_service_ms;
    uint32_t last_imu_service_ms;
    uint8_t can_pending;
    uint8_t imu_pending;
    uint8_t initialized;
} hball_runtime_target_stats_t;

extern volatile hball_runtime_target_stats_t g_hball_runtime_target_stats;

bool hball_runtime_target_init(void);
void hball_runtime_target_tick_isr(void);
void hball_runtime_target_poll(uint32_t now_ms);
uint32_t hball_runtime_target_missed(hball_coop_task_t task);

#ifdef __cplusplus
}
#endif

#endif

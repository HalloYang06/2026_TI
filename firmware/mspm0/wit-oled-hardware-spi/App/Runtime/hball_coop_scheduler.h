#ifndef HBALL_COOP_SCHEDULER_H
#define HBALL_COOP_SCHEDULER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_COOP_MAX_PENDING 3U

typedef enum
{
    HBALL_COOP_TASK_SAFETY = 0,
    HBALL_COOP_TASK_CAN,
    HBALL_COOP_TASK_MISSION,
    HBALL_COOP_TASK_BUTTON,
    HBALL_COOP_TASK_LINE,
    HBALL_COOP_TASK_WHEEL,
    HBALL_COOP_TASK_LCD,
    HBALL_COOP_TASK_COUNT,
} hball_coop_task_t;

typedef struct
{
    volatile uint16_t countdown_ms[HBALL_COOP_TASK_COUNT];
    volatile uint8_t pending[HBALL_COOP_TASK_COUNT];
    volatile uint32_t missed[HBALL_COOP_TASK_COUNT];
} hball_coop_scheduler_t;

void hball_coop_scheduler_init(hball_coop_scheduler_t *scheduler);
void hball_coop_scheduler_tick_isr(hball_coop_scheduler_t *scheduler);

/*
 * The target runtime must call take from a short critical section because
 * SysTick publishes pending work concurrently. Host tests are single-threaded.
 */
bool hball_coop_scheduler_take(
    hball_coop_scheduler_t *scheduler,
    hball_coop_task_t task
);
uint8_t hball_coop_scheduler_pending(
    const hball_coop_scheduler_t *scheduler,
    hball_coop_task_t task
);
uint32_t hball_coop_scheduler_missed(
    const hball_coop_scheduler_t *scheduler,
    hball_coop_task_t task
);
uint16_t hball_coop_scheduler_period_ms(hball_coop_task_t task);

#ifdef __cplusplus
}
#endif

#endif

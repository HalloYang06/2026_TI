#ifndef HBALL_CAN_RECOVERY_H
#define HBALL_CAN_RECOVERY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Limit recovery attempts while a disconnected bus remains recessive. */
#define HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS UINT32_C(1000)

typedef struct
{
    uint32_t last_attempt_ms;
    bool bus_off_active;
} hball_can_recovery_t;

void hball_can_recovery_init(hball_can_recovery_t *recovery);

bool hball_can_recovery_should_attempt(
    hball_can_recovery_t *recovery,
    uint32_t now_ms,
    bool bus_off
);

#ifdef __cplusplus
}
#endif

#endif

#include "hball_can_recovery.h"

void hball_can_recovery_init(hball_can_recovery_t *recovery)
{
    recovery->last_attempt_ms = 0U;
    recovery->bus_off_active = false;
}

bool hball_can_recovery_should_attempt(
    hball_can_recovery_t *recovery,
    uint32_t now_ms,
    bool bus_off
)
{
    if (!bus_off)
    {
        recovery->bus_off_active = false;
        return false;
    }

    if (!recovery->bus_off_active)
    {
        recovery->bus_off_active = true;
        recovery->last_attempt_ms = now_ms;
        return false;
    }

    if ((uint32_t)(now_ms - recovery->last_attempt_ms)
        < HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS)
    {
        return false;
    }

    recovery->last_attempt_ms = now_ms;
    return true;
}

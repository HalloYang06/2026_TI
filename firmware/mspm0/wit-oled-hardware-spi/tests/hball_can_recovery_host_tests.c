#include "hball_can_recovery.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

static void test_bus_off_waits_before_first_recovery_attempt(void)
{
    hball_can_recovery_t recovery;

    hball_can_recovery_init(&recovery);
    assert(!hball_can_recovery_should_attempt(&recovery, 100U, true));
    assert(!hball_can_recovery_should_attempt(
        &recovery,
        100U + HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS - 1U,
        true));
    assert(hball_can_recovery_should_attempt(
        &recovery,
        100U + HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS,
        true));
}

static void test_persistent_bus_off_is_rate_limited(void)
{
    hball_can_recovery_t recovery;

    hball_can_recovery_init(&recovery);
    assert(!hball_can_recovery_should_attempt(&recovery, 10U, true));
    assert(hball_can_recovery_should_attempt(
        &recovery,
        10U + HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS,
        true));
    assert(!hball_can_recovery_should_attempt(
        &recovery,
        10U + HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS + 1U,
        true));
    assert(hball_can_recovery_should_attempt(
        &recovery,
        10U + (2U * HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS),
        true));
}

static void test_healthy_bus_rearms_the_next_fault(void)
{
    hball_can_recovery_t recovery;

    hball_can_recovery_init(&recovery);
    assert(!hball_can_recovery_should_attempt(&recovery, 1000U, true));
    assert(hball_can_recovery_should_attempt(
        &recovery,
        1000U + HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS,
        true));
    assert(!hball_can_recovery_should_attempt(&recovery, 2500U, false));
    assert(!hball_can_recovery_should_attempt(&recovery, 3000U, true));
    assert(hball_can_recovery_should_attempt(
        &recovery,
        3000U + HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS,
        true));
}

static void test_elapsed_time_handles_millisecond_wraparound(void)
{
    hball_can_recovery_t recovery;
    const uint32_t detected_ms = UINT32_MAX - 100U;

    hball_can_recovery_init(&recovery);
    assert(!hball_can_recovery_should_attempt(
        &recovery, detected_ms, true));
    assert(!hball_can_recovery_should_attempt(
        &recovery,
        detected_ms + HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS - 1U,
        true));
    assert(hball_can_recovery_should_attempt(
        &recovery,
        detected_ms + HBALL_CAN_BUS_OFF_RECOVERY_DELAY_MS,
        true));
}

int main(void)
{
    test_bus_off_waits_before_first_recovery_attempt();
    test_persistent_bus_off_is_rate_limited();
    test_healthy_bus_rearms_the_next_fault();
    test_elapsed_time_handles_millisecond_wraparound();
    return 0;
}

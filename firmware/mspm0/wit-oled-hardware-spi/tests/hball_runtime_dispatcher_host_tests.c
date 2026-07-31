#include "hball_runtime_dispatcher.h"

#include <assert.h>
#include <string.h>

typedef struct
{
    bool can_enabled;
    bool imu_enabled;
    uint32_t enter_total;
    uint32_t exit_total;
    uint32_t can_service_total;
    uint32_t last_can_service_ms;
    uint32_t imu_service_total;
    uint32_t last_imu_service_ms;
} fake_runtime_t;

static void fake_enter_critical(void *context)
{
    fake_runtime_t *runtime = (fake_runtime_t *)context;

    runtime->enter_total++;
}

static void fake_exit_critical(void *context)
{
    fake_runtime_t *runtime = (fake_runtime_t *)context;

    runtime->exit_total++;
}

static bool fake_can_enabled(void *context)
{
    return ((fake_runtime_t *)context)->can_enabled;
}

static void fake_can_service(void *context, uint32_t now_ms)
{
    fake_runtime_t *runtime = (fake_runtime_t *)context;

    runtime->can_service_total++;
    runtime->last_can_service_ms = now_ms;
}

static bool fake_imu_enabled(void *context)
{
    return ((fake_runtime_t *)context)->imu_enabled;
}

static void fake_imu_service(void *context, uint32_t now_ms)
{
    fake_runtime_t *runtime = (fake_runtime_t *)context;

    runtime->imu_service_total++;
    runtime->last_imu_service_ms = now_ms;
}

static hball_runtime_dispatcher_hooks_t make_hooks(fake_runtime_t *runtime)
{
    hball_runtime_dispatcher_hooks_t hooks = {
        fake_enter_critical,
        fake_exit_critical,
        fake_can_enabled,
        fake_can_service,
        fake_imu_enabled,
        fake_imu_service,
        runtime,
    };
    return hooks;
}

static void test_single_release_runs_can_and_imu_once_in_foreground(void)
{
    hball_runtime_dispatcher_t dispatcher;
    fake_runtime_t runtime;
    hball_runtime_dispatcher_hooks_t hooks;

    memset(&runtime, 0, sizeof(runtime));
    runtime.can_enabled = true;
    runtime.imu_enabled = true;
    hooks = make_hooks(&runtime);
    assert(hball_runtime_dispatcher_init(&dispatcher, &hooks));

    hball_runtime_dispatcher_tick_isr(&dispatcher);
    hball_runtime_dispatcher_poll(&dispatcher, 101U);

    assert(runtime.enter_total == 1U);
    assert(runtime.exit_total == 1U);
    assert(runtime.can_service_total == 1U);
    assert(runtime.last_can_service_ms == 101U);
    assert(runtime.imu_service_total == 1U);
    assert(runtime.last_imu_service_ms == 101U);
    assert(hball_runtime_dispatcher_pending(
               &dispatcher, HBALL_COOP_TASK_CAN) == 0U);
    assert(hball_runtime_dispatcher_pending(
               &dispatcher, HBALL_COOP_TASK_IMU) == 0U);
}

static void test_backlog_is_coalesced_without_unbounded_catch_up(void)
{
    hball_runtime_dispatcher_t dispatcher;
    fake_runtime_t runtime;
    hball_runtime_dispatcher_hooks_t hooks;
    uint8_t tick;

    memset(&runtime, 0, sizeof(runtime));
    runtime.can_enabled = true;
    runtime.imu_enabled = true;
    hooks = make_hooks(&runtime);
    assert(hball_runtime_dispatcher_init(&dispatcher, &hooks));
    for (tick = 0U; tick < 5U; ++tick)
    {
        hball_runtime_dispatcher_tick_isr(&dispatcher);
    }

    hball_runtime_dispatcher_poll(&dispatcher, 205U);

    assert(runtime.can_service_total == 1U);
    assert(runtime.imu_service_total == 1U);
    assert(hball_runtime_dispatcher_pending(
               &dispatcher, HBALL_COOP_TASK_CAN) == 0U);
    assert(hball_runtime_dispatcher_missed(
               &dispatcher, HBALL_COOP_TASK_CAN) == 4U);
    assert(hball_runtime_dispatcher_missed(
               &dispatcher, HBALL_COOP_TASK_IMU) == 4U);
}

static void test_disabled_services_consume_release_without_business_work(void)
{
    hball_runtime_dispatcher_t dispatcher;
    fake_runtime_t runtime;
    hball_runtime_dispatcher_hooks_t hooks;

    memset(&runtime, 0, sizeof(runtime));
    hooks = make_hooks(&runtime);
    assert(hball_runtime_dispatcher_init(&dispatcher, &hooks));

    hball_runtime_dispatcher_tick_isr(&dispatcher);
    hball_runtime_dispatcher_poll(&dispatcher, 1U);
    assert(runtime.can_service_total == 0U);
    assert(runtime.imu_service_total == 0U);
    assert(hball_runtime_dispatcher_pending(
               &dispatcher, HBALL_COOP_TASK_CAN) == 0U);
    assert(hball_runtime_dispatcher_pending(
               &dispatcher, HBALL_COOP_TASK_IMU) == 0U);

    runtime.can_enabled = true;
    runtime.imu_enabled = true;
    hball_runtime_dispatcher_poll(&dispatcher, 2U);
    assert(runtime.can_service_total == 0U);
    assert(runtime.imu_service_total == 0U);
    hball_runtime_dispatcher_tick_isr(&dispatcher);
    hball_runtime_dispatcher_poll(&dispatcher, 3U);
    assert(runtime.can_service_total == 1U);
    assert(runtime.imu_service_total == 1U);
}

static void test_invalid_configuration_fails_closed(void)
{
    hball_runtime_dispatcher_t dispatcher;
    fake_runtime_t runtime;
    hball_runtime_dispatcher_hooks_t hooks;

    memset(&runtime, 0, sizeof(runtime));
    hooks = make_hooks(&runtime);
    assert(!hball_runtime_dispatcher_init(NULL, &hooks));
    assert(!hball_runtime_dispatcher_init(&dispatcher, NULL));
    hooks.can_service = NULL;
    assert(!hball_runtime_dispatcher_init(&dispatcher, &hooks));
    hooks = make_hooks(&runtime);
    hooks.imu_service = NULL;
    assert(!hball_runtime_dispatcher_init(&dispatcher, &hooks));

    hball_runtime_dispatcher_tick_isr(NULL);
    hball_runtime_dispatcher_poll(NULL, 0U);
    assert(hball_runtime_dispatcher_pending(
               NULL, HBALL_COOP_TASK_CAN) == 0U);
}

int main(void)
{
    test_single_release_runs_can_and_imu_once_in_foreground();
    test_backlog_is_coalesced_without_unbounded_catch_up();
    test_disabled_services_consume_release_without_business_work();
    test_invalid_configuration_fails_closed();
    return 0;
}

#include "hball_runtime_services.h"

#include <stddef.h>
#include <stdint.h>

#define HBALL_RUNTIME_SERVICE_CAN (UINT8_C(1) << 0)
#define HBALL_RUNTIME_SERVICE_IMU (UINT8_C(1) << 1)
#define HBALL_RUNTIME_SERVICE_MENU_MASK \
    (HBALL_RUNTIME_SERVICE_CAN | HBALL_RUNTIME_SERVICE_IMU)

static volatile uint8_t g_hball_runtime_service_mask =
    HBALL_RUNTIME_SERVICE_MENU_MASK;

void hball_runtime_services_enter_menu(void)
{
    g_hball_runtime_service_mask = HBALL_RUNTIME_SERVICE_MENU_MASK;
}

void hball_runtime_services_apply_policy(
    const hball_mission_policy_t *policy
)
{
    uint8_t mask = 0U;

    if (policy != NULL)
    {
        if (policy->can_business_enabled)
        {
            mask |= HBALL_RUNTIME_SERVICE_CAN;
        }
        if (policy->imu_business_enabled)
        {
            mask |= HBALL_RUNTIME_SERVICE_IMU;
        }
    }
    g_hball_runtime_service_mask = mask;
}

bool hball_runtime_services_can_enabled(void)
{
    return (g_hball_runtime_service_mask & HBALL_RUNTIME_SERVICE_CAN) != 0U;
}

bool hball_runtime_services_imu_enabled(void)
{
    return (g_hball_runtime_service_mask & HBALL_RUNTIME_SERVICE_IMU) != 0U;
}

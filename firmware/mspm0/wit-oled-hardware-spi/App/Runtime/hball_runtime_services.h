#ifndef HBALL_RUNTIME_SERVICES_H
#define HBALL_RUNTIME_SERVICES_H

#include "hball_mission_policy.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void hball_runtime_services_enter_menu(void);
void hball_runtime_services_apply_policy(
    const hball_mission_policy_t *policy
);
bool hball_runtime_services_can_enabled(void);
bool hball_runtime_services_imu_enabled(void);

#ifdef __cplusplus
}
#endif

#endif

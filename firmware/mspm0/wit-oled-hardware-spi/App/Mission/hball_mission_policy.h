#ifndef HBALL_MISSION_POLICY_H
#define HBALL_MISSION_POLICY_H

#include "hball_mission_can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    HBALL_MISSION_OWNER_INVALID = 0,
    HBALL_MISSION_OWNER_LOCAL_MSP,
    HBALL_MISSION_OWNER_REMOTE_M33,
} hball_mission_owner_t;

typedef struct
{
    hball_mission_owner_t owner;
    bool local_start;
    bool chassis_allowed;
    bool wait_for_m33_running;
    bool can_business_enabled;
    bool imu_business_enabled;
} hball_mission_policy_t;

bool hball_mission_policy_get(
    uint8_t mission_id,
    hball_mission_policy_t *policy
);

#ifdef __cplusplus
}
#endif

#endif

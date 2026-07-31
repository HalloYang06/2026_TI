#ifndef HBALL_MISSION_POLICY_H
#define HBALL_MISSION_POLICY_H

#include "hball_mission_can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool hball_mission_uses_can(uint8_t mission_id);
bool hball_mission_runs_chassis(uint8_t mission_id);
bool hball_mission_runs_ball_control(uint8_t mission_id);

#ifdef __cplusplus
}
#endif

#endif

#ifndef CHASSIS_MOTION_PROFILE_H
#define CHASSIS_MOTION_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float start_scale;
    float end_scale;
    float current_scale;
    uint32_t start_ms;
    uint32_t duration_ms;
    bool active;
} chassis_motion_profile_t;

void chassis_motion_profile_start(
    chassis_motion_profile_t *profile,
    float start_scale,
    float end_scale,
    uint32_t start_ms,
    uint32_t duration_ms
);
float chassis_motion_profile_sample(
    chassis_motion_profile_t *profile,
    uint32_t now_ms
);
int16_t chassis_motion_profile_scale_i16(int16_t command, float scale);

#endif

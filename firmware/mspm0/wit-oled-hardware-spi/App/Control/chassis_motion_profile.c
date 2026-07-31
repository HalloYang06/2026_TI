#include "chassis_motion_profile.h"

#include <stddef.h>

static float clamp_scale(float scale)
{
    if (scale < 0.0F)
    {
        return 0.0F;
    }
    if (scale > 1.0F)
    {
        return 1.0F;
    }
    return scale;
}

void chassis_motion_profile_start(
    chassis_motion_profile_t *profile,
    float start_scale,
    float end_scale,
    uint32_t start_ms,
    uint32_t duration_ms
)
{
    if (profile == NULL)
    {
        return;
    }
    profile->start_scale = clamp_scale(start_scale);
    profile->end_scale = clamp_scale(end_scale);
    profile->current_scale = profile->start_scale;
    profile->start_ms = start_ms;
    profile->duration_ms = duration_ms;
    profile->active = duration_ms != 0U;
    if (!profile->active)
    {
        profile->current_scale = profile->end_scale;
    }
}

float chassis_motion_profile_sample(
    chassis_motion_profile_t *profile,
    uint32_t now_ms
)
{
    uint32_t elapsed_ms;
    float u;
    float smootherstep;

    if (profile == NULL)
    {
        return 0.0F;
    }
    if (!profile->active)
    {
        return profile->current_scale;
    }
    elapsed_ms = (uint32_t)(now_ms - profile->start_ms);
    if (elapsed_ms >= profile->duration_ms)
    {
        profile->current_scale = profile->end_scale;
        profile->active = false;
        return profile->current_scale;
    }
    u = (float)elapsed_ms / (float)profile->duration_ms;
    smootherstep = u * u * u * (10.0F + u * (-15.0F + 6.0F * u));
    profile->current_scale = profile->start_scale
        + (profile->end_scale - profile->start_scale) * smootherstep;
    return profile->current_scale;
}

int16_t chassis_motion_profile_scale_i16(int16_t command, float scale)
{
    return (int16_t)((float)command * clamp_scale(scale));
}

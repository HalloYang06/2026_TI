#include "chassis_motion_profile.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

int main(void)
{
    chassis_motion_profile_t profile;
    float previous;
    uint32_t elapsed_ms;

    chassis_motion_profile_start(&profile, 0.0F, 1.0F, 100U, 600U);
    assert(fabsf(chassis_motion_profile_sample(&profile, 100U)) < 1.0e-7F);
    assert(fabsf(chassis_motion_profile_sample(&profile, 400U) - 0.5F)
        < 1.0e-6F);
    assert(fabsf(chassis_motion_profile_sample(&profile, 700U) - 1.0F)
        < 1.0e-7F);
    assert(!profile.active);

    chassis_motion_profile_start(&profile, 0.0F, 1.0F, 0U, 600U);
    previous = 0.0F;
    for (elapsed_ms = 0U; elapsed_ms <= 600U; elapsed_ms += 10U)
    {
        const float current = chassis_motion_profile_sample(
            &profile, elapsed_ms
        );
        assert(current >= previous);
        previous = current;
    }

    chassis_motion_profile_start(&profile, 1.0F, 0.0F, 0U, 500U);
    assert(fabsf(chassis_motion_profile_sample(&profile, 250U) - 0.5F)
        < 1.0e-6F);
    assert(fabsf(chassis_motion_profile_sample(&profile, 500U)) < 1.0e-7F);

    chassis_motion_profile_start(
        &profile, 0.0F, 1.0F, UINT32_MAX - 99U, 200U
    );
    assert(fabsf(chassis_motion_profile_sample(&profile, 0U) - 0.5F)
        < 1.0e-6F);

    chassis_motion_profile_start(&profile, 0.0F, 1.0F, 0U, 0U);
    assert(fabsf(chassis_motion_profile_sample(&profile, 0U) - 1.0F)
        < 1.0e-7F);
    assert(chassis_motion_profile_scale_i16(50, 0.5F) == 25);
    assert(chassis_motion_profile_scale_i16(-50, 0.5F) == -25);
    assert(chassis_motion_profile_scale_i16(50, 2.0F) == 50);
    return 0;
}

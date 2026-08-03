#include "hball_imu_compensation.h"

#include <math.h>

float hball_imu_specific_force_to_vehicle_accel(
    float specific_force_forward_mps2,
    float body_pitch_rad
)
{
    const float pitch_cos = cosf(body_pitch_rad);

    if (!isfinite(specific_force_forward_mps2)
        || !isfinite(body_pitch_rad)
        || (fabsf(pitch_cos) < 0.5F))
    {
        return 0.0F;
    }

    /* f_forward = a_forward * cos(pitch) + g * sin(pitch). */
    return (
        specific_force_forward_mps2
        - HBALL_IMU_GRAVITY_MPS2 * sinf(body_pitch_rad)
    ) / pitch_cos;
}

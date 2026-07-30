#ifndef HBALL_FOURBAR_H
#define HBALL_FOURBAR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float motor_pivot_x_m;
    float motor_pivot_y_m;
    float crank_length_m;
    float coupler_length_m;
    float pipe_radius_m;
    float geometric_level_motor_angle_rad;
} hball_fourbar_geometry_t;

void hball_fourbar_default_geometry(hball_fourbar_geometry_t *geometry);
bool hball_fourbar_inverse(
    const hball_fourbar_geometry_t *geometry,
    float pipe_angle_rad,
    float *geometric_motor_angle_rad
);
bool hball_fourbar_forward(
    const hball_fourbar_geometry_t *geometry,
    float geometric_motor_angle_rad,
    float *pipe_angle_rad
);
bool hball_fourbar_motor_offset(
    const hball_fourbar_geometry_t *geometry,
    float pipe_angle_rad,
    float *motor_offset_from_level_rad
);

#ifdef __cplusplus
}
#endif

#endif

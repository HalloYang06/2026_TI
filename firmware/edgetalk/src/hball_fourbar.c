#include "hball_fourbar.h"

#include <math.h>
#include <stddef.h>

#define HBALL_PI_F 3.14159265358979323846F
#define HBALL_TWO_PI_F (2.0F * HBALL_PI_F)
#define HBALL_FOURBAR_EPSILON_M 1.0e-7F

static float hball_wrap_pi(float angle_rad)
{
    while (angle_rad > HBALL_PI_F)
    {
        angle_rad -= HBALL_TWO_PI_F;
    }
    while (angle_rad < -HBALL_PI_F)
    {
        angle_rad += HBALL_TWO_PI_F;
    }
    return angle_rad;
}

void hball_fourbar_default_geometry(hball_fourbar_geometry_t *geometry)
{
    if (geometry == NULL)
    {
        return;
    }
    geometry->motor_pivot_x_m = -0.285F;
    geometry->motor_pivot_y_m = -0.055F;
    geometry->crank_length_m = 0.0350F;
    geometry->coupler_length_m = 0.0555F;
    geometry->pipe_radius_m = 0.3001F;
    geometry->geometric_level_motor_angle_rad = 3.051858578444F;
}

static bool hball_circle_intersection(
    float first_x,
    float first_y,
    float first_radius,
    float second_x,
    float second_y,
    float second_radius,
    float branch,
    float *point_x,
    float *point_y
)
{
    const float dx = second_x - first_x;
    const float dy = second_y - first_y;
    const float distance = hypotf(dx, dy);
    float along;
    float height_squared;
    float base_x;
    float base_y;
    float height;

    if ((point_x == NULL) || (point_y == NULL)
        || !isfinite(distance) || (distance < HBALL_FOURBAR_EPSILON_M)
        || (distance > first_radius + second_radius)
        || (distance < fabsf(first_radius - second_radius)))
    {
        return false;
    }
    along = (
        first_radius * first_radius
        - second_radius * second_radius
        + distance * distance
    ) / (2.0F * distance);
    height_squared = first_radius * first_radius - along * along;
    if (height_squared < -HBALL_FOURBAR_EPSILON_M)
    {
        return false;
    }
    if (height_squared < 0.0F)
    {
        height_squared = 0.0F;
    }
    height = sqrtf(height_squared);
    base_x = first_x + along * dx / distance;
    base_y = first_y + along * dy / distance;
    *point_x = base_x + branch * height * (-dy) / distance;
    *point_y = base_y + branch * height * dx / distance;
    return isfinite(*point_x) && isfinite(*point_y);
}

bool hball_fourbar_inverse(
    const hball_fourbar_geometry_t *geometry,
    float pipe_angle_rad,
    float *geometric_motor_angle_rad
)
{
    float point_a_x;
    float point_a_y;
    float point_b_x;
    float point_b_y;

    if ((geometry == NULL) || (geometric_motor_angle_rad == NULL)
        || !isfinite(pipe_angle_rad))
    {
        return false;
    }
    point_b_x = -geometry->pipe_radius_m * cosf(pipe_angle_rad);
    point_b_y = -geometry->pipe_radius_m * sinf(pipe_angle_rad);
    if (!hball_circle_intersection(
            geometry->motor_pivot_x_m,
            geometry->motor_pivot_y_m,
            geometry->crank_length_m,
            point_b_x,
            point_b_y,
            geometry->coupler_length_m,
            1.0F,
            &point_a_x,
            &point_a_y))
    {
        return false;
    }
    *geometric_motor_angle_rad = atan2f(
        point_a_y - geometry->motor_pivot_y_m,
        point_a_x - geometry->motor_pivot_x_m
    );
    return isfinite(*geometric_motor_angle_rad);
}

bool hball_fourbar_forward(
    const hball_fourbar_geometry_t *geometry,
    float geometric_motor_angle_rad,
    float *pipe_angle_rad
)
{
    float point_a_x;
    float point_a_y;
    float point_b_x;
    float point_b_y;

    if ((geometry == NULL) || (pipe_angle_rad == NULL)
        || !isfinite(geometric_motor_angle_rad))
    {
        return false;
    }
    point_a_x = geometry->motor_pivot_x_m
        + geometry->crank_length_m * cosf(geometric_motor_angle_rad);
    point_a_y = geometry->motor_pivot_y_m
        + geometry->crank_length_m * sinf(geometric_motor_angle_rad);
    if (!hball_circle_intersection(
            0.0F,
            0.0F,
            geometry->pipe_radius_m,
            point_a_x,
            point_a_y,
            geometry->coupler_length_m,
            -1.0F,
            &point_b_x,
            &point_b_y))
    {
        return false;
    }
    *pipe_angle_rad = hball_wrap_pi(
        atan2f(point_b_y, point_b_x) - HBALL_PI_F
    );
    return isfinite(*pipe_angle_rad);
}

bool hball_fourbar_motor_offset(
    const hball_fourbar_geometry_t *geometry,
    float pipe_angle_rad,
    float *motor_offset_from_level_rad
)
{
    float motor_angle_rad;

    if ((geometry == NULL) || (motor_offset_from_level_rad == NULL)
        || !hball_fourbar_inverse(geometry, pipe_angle_rad, &motor_angle_rad))
    {
        return false;
    }
    *motor_offset_from_level_rad = hball_wrap_pi(
        motor_angle_rad - geometry->geometric_level_motor_angle_rad
    );
    return isfinite(*motor_offset_from_level_rad);
}

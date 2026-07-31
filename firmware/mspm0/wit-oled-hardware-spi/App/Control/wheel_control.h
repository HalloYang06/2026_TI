#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

#include "motion_intent.h"
#include "pid.h"

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_CONTROL_PERIOD_MS 100U

typedef struct
{
    PID_t left_pid;
    PID_t right_pid;
    int32_t previous_left_count;
    int32_t previous_right_count;
    int16_t commanded_duty_left;
    int16_t commanded_duty_right;
    uint32_t last_update_ms;
} wheel_control_t;

typedef struct
{
    int32_t measured_left_speed;
    int32_t measured_right_speed;
    int16_t duty_left;
    int16_t duty_right;
} wheel_control_output_t;

void wheel_control_init(
    wheel_control_t *control,
    uint32_t now_ms,
    int16_t initial_duty_left,
    int16_t initial_duty_right
);
bool wheel_control_due(const wheel_control_t *control, uint32_t now_ms);
bool wheel_control_step(
    wheel_control_t *control,
    uint32_t now_ms,
    int32_t encoder_left_count,
    int32_t encoder_right_count,
    const motion_intent_t *intent,
    wheel_control_output_t *output
);
void wheel_control_reset_integrators(wheel_control_t *control);

#endif

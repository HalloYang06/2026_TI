#ifndef HBALL_CHASSIS_ACTUATOR_H
#define HBALL_CHASSIS_ACTUATOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    CHASSIS_WHEEL_LEFT = 1U,
    CHASSIS_WHEEL_RIGHT = 2U,
} chassis_wheel_t;

void chassis_actuator_init(void);
void chassis_actuator_stop(void);
void chassis_actuator_enable(void);
void chassis_actuator_disable(void);
void chassis_actuator_start_synchronized(float left_pwm, float right_pwm);
void chassis_actuator_set_wheel_speed(float duty, uint8_t wheel);
void chassis_actuator_set_pwm(float left_pwm, float right_pwm);

#ifdef __cplusplus
}
#endif

#endif

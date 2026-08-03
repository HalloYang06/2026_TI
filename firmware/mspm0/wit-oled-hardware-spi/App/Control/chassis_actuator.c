#include "chassis_actuator.h"

#include "motor.h"

void chassis_actuator_init(void)
{
    motor_init();
}

void chassis_actuator_stop(void)
{
    motor_stop();
}

void chassis_actuator_enable(void)
{
    motor_driver_enable();
}

void chassis_actuator_disable(void)
{
    motor_driver_disable();
}

void chassis_actuator_start_synchronized(float left_pwm, float right_pwm)
{
    motor_start_synchronized(left_pwm, right_pwm);
}

void chassis_actuator_set_wheel_speed(float duty, uint8_t wheel)
{
    set_motor_speed(duty, wheel);
}

void chassis_actuator_set_pwm(float left_pwm, float right_pwm)
{
    motor_pwm_set(left_pwm, right_pwm);
}

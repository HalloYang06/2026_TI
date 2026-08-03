#ifndef __motor_h
#define __motor_h
#include "ti_msp_dl_config.h"
#define left_motor  1
#define right_motor 2
#define MAX_SPEED_UP 3.0
void motor_stop(void);
void motor_init(void);
void motor_driver_enable(void);
void motor_driver_disable(void);
void motor_start_synchronized(float pwm1,float pwm2);
void set_motor_speed(float duty,uint8_t motor);
void pwm_limiting(int *pwm1,int *pwm2);
void left_motor_dir(uint8_t para);
void right_motor_dir(uint8_t para);
void motor_pwm_set(float pwm1,float pwm2);

#endif

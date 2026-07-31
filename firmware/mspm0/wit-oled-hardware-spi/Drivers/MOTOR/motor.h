#ifndef __motor_h
#define __motor_h
#include "ti_msp_dl_config.h"
#define AIN1_SET   DL_GPIO_setPins(motor_gpio_PORT,motor_gpio_AIN1_PIN);
#define AIN1_RESET   DL_GPIO_clearPins(motor_gpio_PORT,motor_gpio_AIN1_PIN);
#define AIN2_SET   DL_GPIO_setPins(motor_gpio_PORT,motor_gpio_AIN2_PIN);
#define AIN2_RESET   DL_GPIO_clearPins(motor_gpio_PORT,motor_gpio_AIN2_PIN);
#define BIN1_SET   DL_GPIO_setPins(motor_gpio_PORT,motor_gpio_BIN1_PIN);
#define BIN1_RESET   DL_GPIO_clearPins(motor_gpio_PORT,motor_gpio_BIN1_PIN);
#define BIN2_SET   DL_GPIO_setPins(motor_gpio_PORT,motor_gpio_BIN2_PIN);
#define BIN2_RESET   DL_GPIO_clearPins(motor_gpio_PORT,motor_gpio_BIN2_PIN);
#define left_motor  1
#define right_motor 2
#define MAX_SPEED_UP 3.0
void motor_stop(void);
void motor_init(void);
void motor_start_synchronized(float pwm1,float pwm2);
void set_motor_speed(float duty,uint8_t motor);
void pwm_limiting(int *pwm1,int *pwm2);
void left_motor_dir(uint8_t para);
void right_motor_dir(uint8_t para);
void motor_pwm_set(float pwm1,float pwm2);

#endif

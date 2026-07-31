#include "motor.h"

float compareval=0;
int led_state=0;
void motor_init(void)
{
	motor_stop();
}
void motor_stop(void)
{
    
    AIN1_RESET;
    AIN2_RESET;

    BIN1_RESET;
    BIN2_RESET;
    
}

void set_motor_speed(float duty,uint8_t motor)
{
    if (duty < 0.0f) {
        duty = 0.0f;
    } else if (duty > 100.0f) {
        duty = 100.0f;
    }

    if(motor==1)
    {
        compareval=9999-9999*(duty/100.0f);
         DL_TimerA_setCaptureCompareValue(PWM_A_INST,compareval,GPIO_PWM_A_C0_IDX);
    }
    if(motor==2)
    {
        compareval=9999-9999*(duty/100.0f);
         DL_TimerA_setCaptureCompareValue(PWM_A_INST,compareval,GPIO_PWM_A_C1_IDX);
    }
  
}
void pwm_limiting(int *pwm1,int *pwm2)
{
    int max=99;
    if(*pwm1>max){*pwm1=max;}
    if(*pwm1<-max){*pwm1=-max;}
    if(*pwm2>max){*pwm2=max;}
    if(*pwm2<-max){*pwm2=-max;}
}
void left_motor_dir(uint8_t para)
{
    if(para==1)
    {
        AIN1_SET;
        AIN2_RESET;
    }
    else
    {
        AIN1_RESET;
        AIN2_SET;
    }
}

void right_motor_dir(uint8_t para)
{
    if(para==1)
    {
        BIN1_RESET;
        BIN2_SET;
    }
    else
    {
        BIN1_SET;
        BIN2_RESET;
    }
}
void motor_pwm_set(float pwm1,float pwm2)
{
    int val1 = (int )pwm1;
    int val2 = (int )-pwm2;
    pwm_limiting(&val1,&val2);

    if (val1 >= 0)
    {
        left_motor_dir(1);
        set_motor_speed((float)val1, (uint8_t)left_motor);
    }
    else
    {
        left_motor_dir(0);
        set_motor_speed((float)-val1, (uint8_t)left_motor);
    }

    if (val2 >= 0)
    {
        right_motor_dir(1);
        set_motor_speed((float)val2, (uint8_t)right_motor);
    }
    else
    {
        right_motor_dir(0);
        set_motor_speed((float)-val2, (uint8_t)right_motor);
    }
}

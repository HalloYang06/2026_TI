#include "key.h"

int get_keynum()
{
    int number1=0;
    if(DL_GPIO_readPins(GPIO_KEY_PORT,GPIO_KEY_KEY_2_PIN)==0)
    {   
        delay_cycles(200);
        if(DL_GPIO_readPins(GPIO_KEY_PORT,GPIO_KEY_KEY_2_PIN)==0)
        {
            number1=1;
            while (DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_KEY_2_PIN) == 0);
            delay_cycles(200);
        }
    }
    if(DL_GPIO_readPins(GPIO_KEY_PORT,GPIO_KEY_KEY_1_PIN)==0)
    {
         delay_cycles(200);
        if(DL_GPIO_readPins(GPIO_KEY_PORT,GPIO_KEY_KEY_1_PIN)==0)
        {
            number1=2;
             // 等待按键释放
            while (DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_KEY_1_PIN) == 0);
            delay_cycles(200);
        }
    }
   if(DL_GPIO_readPins(GPIO_KEY_PORT,GPIO_KEY_KEY_3_PIN)==0)
    {
         delay_cycles(200);
        if(DL_GPIO_readPins(GPIO_KEY_PORT,GPIO_KEY_KEY_3_PIN)==0)
        {
            number1=3;
            while (DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_KEY_3_PIN) == 0);
           delay_cycles(200);
        }
    }
    return number1;
}


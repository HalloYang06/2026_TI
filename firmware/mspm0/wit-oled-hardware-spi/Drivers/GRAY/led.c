#include "led.h"


void flash(void)
{
    DL_GPIO_setPins(GPIO_LED_PORT,GPIO_LED_PIN_8_PIN);
    delay_cycles(1000000);
    DL_GPIO_clearPins(GPIO_LED_PORT,GPIO_LED_PIN_8_PIN);
}
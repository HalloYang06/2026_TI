#include"beeper.h"

void beep(void)
{
    DL_GPIO_clearPins(GPIO_BEEPER_PORT, GPIO_BEEPER_PIN_20_PIN);
    delay_cycles(500000);
    DL_GPIO_setPins(GPIO_BEEPER_PORT, GPIO_BEEPER_PIN_20_PIN);
}

void clear(void)
{
    DL_GPIO_setPins(GPIO_BEEPER_PORT, GPIO_BEEPER_PIN_20_PIN);
}

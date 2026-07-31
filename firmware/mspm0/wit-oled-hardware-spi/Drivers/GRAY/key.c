#include "key.h"

#define TASK_KEY_DEBOUNCE_CYCLES (CPUCLK_FREQ / 50U)

static uint8_t task_key_latched;

static uint8_t task_key_is_pressed(uint32_t pin)
{
    return (DL_GPIO_readPins(GPIO_KEY_PORT, pin) == 0U) ? 1U : 0U;
}

task_key_event_t get_task_key_event(void)
{
    uint8_t select_pressed = task_key_is_pressed(TASK_KEY_SELECT_PIN);
    uint8_t execute_pressed = task_key_is_pressed(TASK_KEY_EXECUTE_PIN);
    task_key_event_t event;

    if (task_key_latched != 0U)
    {
        if ((select_pressed != 0U) || (execute_pressed != 0U))
        {
            return TASK_KEY_EVENT_NONE;
        }
        delay_cycles(TASK_KEY_DEBOUNCE_CYCLES);
        if ((task_key_is_pressed(TASK_KEY_SELECT_PIN) == 0U)
            && (task_key_is_pressed(TASK_KEY_EXECUTE_PIN) == 0U))
        {
            task_key_latched = 0U;
        }
        return TASK_KEY_EVENT_NONE;
    }

    if ((select_pressed == 0U) && (execute_pressed == 0U)) {
        return TASK_KEY_EVENT_NONE;
    }

    delay_cycles(TASK_KEY_DEBOUNCE_CYCLES);
    select_pressed = task_key_is_pressed(TASK_KEY_SELECT_PIN);
    execute_pressed = task_key_is_pressed(TASK_KEY_EXECUTE_PIN);

    /* Ignore an ambiguous two-button press until a clean release. */
    if (select_pressed == execute_pressed)
    {
        task_key_latched = 1U;
        return TASK_KEY_EVENT_NONE;
    }

    if (select_pressed != 0U) {
        event = TASK_KEY_EVENT_SELECT;
    } else {
        event = TASK_KEY_EVENT_EXECUTE;
    }

    task_key_latched = 1U;
    return event;
}

int get_keynum(void)
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

#include "FreeRTOS.h"
#include "task.h"

#include "cybsp.h"
#include "hball_m55_freertos.h"

static void hball_m55_fail_closed(void)
{
    __disable_irq();
    for (;;)
    {
        __WFI();
    }
}

int main(void)
{
    if (cybsp_init() != CY_RSLT_SUCCESS)
    {
        hball_m55_fail_closed();
    }
    __enable_irq();
    if (hball_m55_freertos_start() != pdPASS)
    {
        hball_m55_fail_closed();
    }
    vTaskStartScheduler();
    hball_m55_fail_closed();
    return 0;
}

void vApplicationMallocFailedHook(void)
{
    hball_m55_fail_closed();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void)task;
    (void)task_name;
    hball_m55_fail_closed();
}

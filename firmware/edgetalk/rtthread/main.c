#include <board.h>
#include <reent.h>
#include <rtdevice.h>
#include <rtthread.h>

#ifndef HBALL_USB_ONLY
#define HBALL_USB_ONLY 0
#endif

#define HBALL_HEARTBEAT_LED GET_PIN(16, 5)
#define HBALL_HEARTBEAT_PERIOD_MS 500U

__attribute__((weak)) struct _reent _impure_data;

int main(void)
{
    rt_base_t heartbeat_level = PIN_LOW;

#if HBALL_USB_ONLY
    rt_kprintf("[hball] EdgeTalk M33 USB-only CDC probe main\n");
#else
    rt_kprintf("[hball] EdgeTalk M33 read-only CAN bench main\n");
#endif
    rt_pin_mode(HBALL_HEARTBEAT_LED, PIN_MODE_OUTPUT);
    rt_pin_write(HBALL_HEARTBEAT_LED, heartbeat_level);
    while (1)
    {
        heartbeat_level = (heartbeat_level == PIN_LOW) ? PIN_HIGH : PIN_LOW;
        rt_pin_write(HBALL_HEARTBEAT_LED, heartbeat_level);
        rt_thread_mdelay(HBALL_HEARTBEAT_PERIOD_MS);
    }
}

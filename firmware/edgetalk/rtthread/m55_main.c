#include <reent.h>
#include <rtthread.h>

__attribute__((weak)) struct _reent _impure_data;

int main(void)
{
    rt_kprintf("[hball-m55] EdgeTalk M55 LQG shadow main\n");
    while (1)
    {
        rt_thread_mdelay(1000U);
    }
}

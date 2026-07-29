#include "hball_rate_meter.h"

#include <stddef.h>
#include <string.h>

void hball_rate_meter_init(hball_rate_meter_t *meter)
{
    if (meter != NULL)
    {
        memset(meter, 0, sizeof(*meter));
    }
}

void hball_rate_meter_accept(hball_rate_meter_t *meter, uint32_t now_ms)
{
    if (meter == NULL)
    {
        return;
    }
    if (!meter->initialized)
    {
        meter->first_event_ms = now_ms;
        meter->initialized = true;
    }
    meter->last_event_ms = now_ms;
    meter->event_total++;
}

uint32_t hball_rate_meter_hz_x10(
    const hball_rate_meter_t *meter, uint32_t now_ms
)
{
    uint32_t elapsed_ms;
    uint64_t intervals_x10000;

    if ((meter == NULL) || !meter->initialized || (meter->event_total < 2U))
    {
        return 0U;
    }
    elapsed_ms = (uint32_t)(now_ms - meter->first_event_ms);
    if (elapsed_ms == 0U)
    {
        return 0U;
    }
    intervals_x10000 = (uint64_t)(meter->event_total - 1U) * UINT64_C(10000);
    return (uint32_t)(intervals_x10000 / elapsed_ms);
}

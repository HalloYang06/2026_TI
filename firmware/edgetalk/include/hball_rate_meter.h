#ifndef HBALL_RATE_METER_H
#define HBALL_RATE_METER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t first_event_ms;
    uint32_t last_event_ms;
    uint32_t event_total;
    bool initialized;
} hball_rate_meter_t;

void hball_rate_meter_init(hball_rate_meter_t *meter);
void hball_rate_meter_accept(hball_rate_meter_t *meter, uint32_t now_ms);
uint32_t hball_rate_meter_hz_x10(
    const hball_rate_meter_t *meter, uint32_t now_ms
);

#ifdef __cplusplus
}
#endif

#endif

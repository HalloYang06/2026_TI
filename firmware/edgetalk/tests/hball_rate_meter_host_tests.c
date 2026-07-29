#include "hball_rate_meter.h"

#include <assert.h>
#include <stdint.h>

static void test_empty_and_single_event_have_no_rate(void)
{
    hball_rate_meter_t meter;

    hball_rate_meter_init(&meter);
    assert(hball_rate_meter_hz_x10(&meter, 1000U) == 0U);
    hball_rate_meter_accept(&meter, 1000U);
    assert(hball_rate_meter_hz_x10(&meter, 1000U) == 0U);
}

static void test_rate_uses_intervals_and_wrap_safe_time(void)
{
    hball_rate_meter_t meter;
    uint32_t index;

    hball_rate_meter_init(&meter);
    for (index = 0U; index <= 200U; ++index)
    {
        hball_rate_meter_accept(&meter, 1000U + index * 5U);
    }
    assert(meter.event_total == 201U);
    assert(hball_rate_meter_hz_x10(&meter, 2000U) == 2000U);

    hball_rate_meter_init(&meter);
    hball_rate_meter_accept(&meter, UINT32_MAX - 4U);
    hball_rate_meter_accept(&meter, 5U);
    assert(hball_rate_meter_hz_x10(&meter, 5U) == 1000U);
}

static void test_rate_decays_when_stream_stops(void)
{
    hball_rate_meter_t meter;

    hball_rate_meter_init(&meter);
    hball_rate_meter_accept(&meter, 0U);
    hball_rate_meter_accept(&meter, 5U);
    assert(hball_rate_meter_hz_x10(&meter, 5U) == 2000U);
    assert(hball_rate_meter_hz_x10(&meter, 1005U) == 9U);
}

int main(void)
{
    test_empty_and_single_event_have_no_rate();
    test_rate_uses_intervals_and_wrap_safe_time();
    test_rate_decays_when_stream_stops();
    return 0;
}

#include "line_sensor_port.h"

#include <assert.h>
#include <stdint.h>

static uint8_t g_pin_high_mask;
static uint8_t g_read_count;

uint32_t line_sensor_test_gpio_read(uintptr_t channel, uint32_t pin)
{
    assert(channel < 8U);
    assert(pin == 1U);
    g_read_count++;
    return (g_pin_high_mask & (uint8_t)(1U << channel)) != 0U ? 1U : 0U;
}

static void test_all_gpio_combinations_preserve_raw_bit_order(void)
{
    uint16_t value;

    for (value = 0U; value <= UINT8_MAX; value++)
    {
        g_pin_high_mask = (uint8_t)value;
        g_read_count = 0U;

        assert(line_sensor_port_read_raw() == (uint8_t)value);
        assert(g_read_count == 8U);
    }
}

static void test_active_mask_is_the_inverse_of_physical_high_levels(void)
{
    uint16_t value;

    for (value = 0U; value <= UINT8_MAX; value++)
    {
        g_pin_high_mask = (uint8_t)value;
        g_read_count = 0U;

        assert(line_sensor_port_read_active_mask() == (uint8_t)(~value));
        assert(g_read_count == 8U);
    }
}

int main(void)
{
    test_all_gpio_combinations_preserve_raw_bit_order();
    test_active_mask_is_the_inverse_of_physical_high_levels();
    return 0;
}

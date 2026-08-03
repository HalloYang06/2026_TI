#include "line_sensor_port.h"

#include "ti_msp_dl_config.h"

uint8_t line_sensor_port_read_raw(void)
{
    uint8_t raw = 0U;

    if (DL_GPIO_readPins(track_PIN_0_PORT, track_PIN_0_PIN) != 0U)
    {
        raw |= (uint8_t)(1U << 0U);
    }
    if (DL_GPIO_readPins(track_PIN_1_PORT, track_PIN_1_PIN) != 0U)
    {
        raw |= (uint8_t)(1U << 1U);
    }
    if (DL_GPIO_readPins(track_PIN_2_PORT, track_PIN_2_PIN) != 0U)
    {
        raw |= (uint8_t)(1U << 2U);
    }
    if (DL_GPIO_readPins(track_PIN_3_PORT, track_PIN_3_PIN) != 0U)
    {
        raw |= (uint8_t)(1U << 3U);
    }
    if (DL_GPIO_readPins(track_PIN_4_PORT, track_PIN_4_PIN) != 0U)
    {
        raw |= (uint8_t)(1U << 4U);
    }
    if (DL_GPIO_readPins(track_PIN_5_PORT, track_PIN_5_PIN) != 0U)
    {
        raw |= (uint8_t)(1U << 5U);
    }
    if (DL_GPIO_readPins(track_PIN_6_PORT, track_PIN_6_PIN) != 0U)
    {
        raw |= (uint8_t)(1U << 6U);
    }
    if (DL_GPIO_readPins(track_PIN_7_PORT, track_PIN_7_PIN) != 0U)
    {
        raw |= (uint8_t)(1U << 7U);
    }

    return raw;
}

uint8_t line_sensor_port_read_active_mask(void)
{
    return (uint8_t)(~line_sensor_port_read_raw());
}

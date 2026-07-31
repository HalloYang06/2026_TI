#ifndef TEST_FAKE_TI_MSP_DL_CONFIG_H
#define TEST_FAKE_TI_MSP_DL_CONFIG_H

#include <stdint.h>

uint32_t line_sensor_test_gpio_read(uintptr_t channel, uint32_t pin);

#define DL_GPIO_readPins(port, pin) \
    line_sensor_test_gpio_read((uintptr_t)(port), (uint32_t)(pin))

#define track_PIN_0_PORT ((uintptr_t)0U)
#define track_PIN_1_PORT ((uintptr_t)1U)
#define track_PIN_2_PORT ((uintptr_t)2U)
#define track_PIN_3_PORT ((uintptr_t)3U)
#define track_PIN_4_PORT ((uintptr_t)4U)
#define track_PIN_5_PORT ((uintptr_t)5U)
#define track_PIN_6_PORT ((uintptr_t)6U)
#define track_PIN_7_PORT ((uintptr_t)7U)

#define track_PIN_0_PIN UINT32_C(1)
#define track_PIN_1_PIN UINT32_C(1)
#define track_PIN_2_PIN UINT32_C(1)
#define track_PIN_3_PIN UINT32_C(1)
#define track_PIN_4_PIN UINT32_C(1)
#define track_PIN_5_PIN UINT32_C(1)
#define track_PIN_6_PIN UINT32_C(1)
#define track_PIN_7_PIN UINT32_C(1)

#endif

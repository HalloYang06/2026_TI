#ifndef LINE_SENSOR_PORT_H
#define LINE_SENSOR_PORT_H

#include <stdint.h>

uint8_t line_sensor_port_read_raw(void);
uint8_t line_sensor_port_read_active_mask(void);

#endif

#ifndef HBALL_M33_INPUTS_H
#define HBALL_M33_INPUTS_H

#include "hball_sensor_fusion.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool hball_m33_inputs_publish_vision(
    const hball_vision_measurement_t *measurement, uint32_t receive_ms
);
bool hball_m33_inputs_publish_msp(const hball_msp_monitor_t *monitor);
bool hball_m33_inputs_publish_motor(
    const hball_motor_feedback_t *feedback, uint32_t receive_ms
);
bool hball_m33_inputs_publish_motor_parameters(
    const hball_motor_parameters_t *parameters
);
bool hball_m33_inputs_get_snapshot(hball_sensor_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif

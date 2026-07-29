#ifndef HBALL_M55_INPUT_H
#define HBALL_M55_INPUT_H

#include "hball_sensor_fusion.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool hball_m55_read_sensor_snapshot(hball_sensor_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif

#include "hball_m55_input.h"

#include <stddef.h>

__attribute__((weak)) bool hball_m55_read_sensor_snapshot(
    hball_sensor_snapshot_t *snapshot
)
{
    (void)snapshot;
    return false;
}

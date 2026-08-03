#include "line_snapshot.h"

#include <stddef.h>

line_snapshot_t line_snapshot_decode(uint8_t raw, uint32_t timestamp_ms)
{
    static const int8_t weights[8] = {
        -35, -25, -15, -5, 5, 15, 25, 35
    };
    line_snapshot_t snapshot = {0};
    uint8_t index;

    snapshot.timestamp_ms = timestamp_ms;
    snapshot.raw = raw;
    snapshot.line_mask = (uint8_t)(~raw);

    for (index = 0U; index < 8U; index++)
    {
        if ((snapshot.line_mask & (uint8_t)(1U << index)) != 0U)
        {
            snapshot.active_count++;
            snapshot.weighted_sum += weights[index];
        }
    }

    snapshot.valid = (snapshot.active_count != 0U);
    if (snapshot.valid)
    {
        snapshot.weighted_error =
            snapshot.weighted_sum / (int16_t)snapshot.active_count;
    }

    return snapshot;
}

bool line_snapshot_has_adjacent(const line_snapshot_t *snapshot,
                                uint8_t width)
{
    uint8_t adjacent;
    uint8_t shift;

    if ((snapshot == NULL) || (width == 0U) || (width > 8U))
    {
        return false;
    }

    adjacent = snapshot->line_mask;
    for (shift = 1U; shift < width; shift++)
    {
        adjacent &= (uint8_t)(snapshot->line_mask >> shift);
    }

    return adjacent != 0U;
}

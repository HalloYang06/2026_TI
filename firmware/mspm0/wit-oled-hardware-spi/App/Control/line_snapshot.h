#ifndef LINE_SNAPSHOT_H
#define LINE_SNAPSHOT_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    bool valid;
    uint32_t timestamp_ms;
    uint8_t raw;
    uint8_t line_mask;
    uint8_t active_count;
    int16_t weighted_sum;
    int16_t weighted_error;
} line_snapshot_t;

line_snapshot_t line_snapshot_decode(uint8_t raw, uint32_t timestamp_ms);
bool line_snapshot_has_adjacent(const line_snapshot_t *snapshot,
                                uint8_t width);

#endif

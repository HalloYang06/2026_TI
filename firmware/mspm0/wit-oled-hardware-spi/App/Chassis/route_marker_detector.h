#ifndef ROUTE_MARKER_DETECTOR_H
#define ROUTE_MARKER_DETECTOR_H

#include "line_snapshot.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint8_t marker_active_threshold;
    uint8_t marker_adjacent_width;
    uint8_t start_clear_max_active;
    uint32_t start_clear_confirm_ms;
    uint32_t marker_min_elapsed_ms;
    uint32_t marker_confirm_ms;
} route_marker_detector_config_t;

typedef struct
{
    route_marker_detector_config_t config;
    uint32_t run_start_ms;
    uint32_t start_clear_candidate_ms;
    uint32_t marker_candidate_ms;
    bool start_clear_candidate_active;
    bool marker_candidate_active;
    bool start_cleared;
    bool marker_confirmed;
} route_marker_detector_t;

typedef struct
{
    uint32_t timestamp_ms;
    bool marker_present;
    bool start_cleared;
    bool start_cleared_event;
    bool force_straight;
    bool marker_confirmed;
    bool marker_confirmed_event;
} route_marker_detector_output_t;

bool route_marker_detector_init(
    route_marker_detector_t *detector,
    const route_marker_detector_config_t *config,
    uint32_t run_start_ms
);
bool route_marker_detector_step(
    route_marker_detector_t *detector,
    const line_snapshot_t *sample,
    route_marker_detector_output_t *output
);

#endif

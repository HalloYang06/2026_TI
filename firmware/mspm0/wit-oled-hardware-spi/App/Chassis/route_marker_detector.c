#include "route_marker_detector.h"

#include <stddef.h>
#include <string.h>

static bool route_marker_config_valid(
    const route_marker_detector_config_t *config
)
{
    return (config != NULL)
        && (config->marker_active_threshold > 0U)
        && (config->marker_active_threshold <= 8U)
        && (config->marker_adjacent_width <= 8U)
        && (config->start_clear_max_active > 0U)
        && (config->start_clear_max_active <= 8U);
}

static bool route_marker_present(
    const route_marker_detector_config_t *config,
    const line_snapshot_t *sample
)
{
    if (sample->active_count < config->marker_active_threshold)
    {
        return false;
    }
    if (config->marker_adjacent_width == 0U)
    {
        return true;
    }
    return line_snapshot_has_adjacent(
        sample, config->marker_adjacent_width);
}

bool route_marker_detector_init(
    route_marker_detector_t *detector,
    const route_marker_detector_config_t *config,
    uint32_t run_start_ms
)
{
    if ((detector == NULL) || !route_marker_config_valid(config))
    {
        return false;
    }

    memset(detector, 0, sizeof(*detector));
    detector->config = *config;
    detector->run_start_ms = run_start_ms;
    return true;
}

bool route_marker_detector_step(
    route_marker_detector_t *detector,
    const line_snapshot_t *sample,
    route_marker_detector_output_t *output
)
{
    bool marker_present;
    bool start_clear_eligible;
    bool start_cleared_event = false;
    bool marker_confirmed_event = false;
    uint32_t elapsed_ms;

    if ((detector == NULL) || (sample == NULL) || (output == NULL))
    {
        return false;
    }

    marker_present = route_marker_present(&detector->config, sample);
    start_clear_eligible =
        (sample->active_count > 0U)
        && (sample->active_count <=
            detector->config.start_clear_max_active)
        && !marker_present;

    if (!detector->start_cleared)
    {
        if (!start_clear_eligible)
        {
            detector->start_clear_candidate_active = false;
        }
        else if (!detector->start_clear_candidate_active)
        {
            detector->start_clear_candidate_active = true;
            detector->start_clear_candidate_ms = sample->timestamp_ms;
        }
        else if ((uint32_t)(sample->timestamp_ms -
                            detector->start_clear_candidate_ms) >=
                 detector->config.start_clear_confirm_ms)
        {
            detector->start_cleared = true;
            detector->start_clear_candidate_active = false;
            start_cleared_event = true;
        }
    }

    elapsed_ms = (uint32_t)(sample->timestamp_ms - detector->run_start_ms);
    if (!detector->marker_confirmed)
    {
        if (!marker_present
            || !detector->start_cleared
            || (elapsed_ms < detector->config.marker_min_elapsed_ms))
        {
            detector->marker_candidate_active = false;
        }
        else if (detector->config.marker_confirm_ms == 0U)
        {
            detector->marker_confirmed = true;
            marker_confirmed_event = true;
        }
        else if (!detector->marker_candidate_active)
        {
            detector->marker_candidate_active = true;
            detector->marker_candidate_ms = sample->timestamp_ms;
        }
        else if ((uint32_t)(sample->timestamp_ms -
                            detector->marker_candidate_ms) >=
                 detector->config.marker_confirm_ms)
        {
            detector->marker_confirmed = true;
            detector->marker_candidate_active = false;
            marker_confirmed_event = true;
        }
    }

    memset(output, 0, sizeof(*output));
    output->timestamp_ms = sample->timestamp_ms;
    output->marker_present = marker_present;
    output->start_cleared = detector->start_cleared;
    output->start_cleared_event = start_cleared_event;
    output->force_straight = marker_present && !detector->start_cleared;
    output->marker_confirmed = detector->marker_confirmed;
    output->marker_confirmed_event = marker_confirmed_event;
    return true;
}

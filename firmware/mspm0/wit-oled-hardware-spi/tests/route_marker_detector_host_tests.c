#include "line_snapshot.h"
#include "route_marker_detector.h"

#include <assert.h>
#include <stdint.h>

static line_snapshot_t sample(uint8_t active_mask, uint32_t now_ms)
{
    return line_snapshot_decode((uint8_t)(~active_mask), now_ms);
}

static route_marker_detector_config_t lap_config(void)
{
    route_marker_detector_config_t config;

    config.marker_active_threshold = 3U;
    config.marker_adjacent_width = 3U;
    config.start_clear_max_active = 4U;
    config.start_clear_confirm_ms = 120U;
    config.marker_min_elapsed_ms = 18000U;
    config.marker_confirm_ms = 20U;
    return config;
}

static void test_start_marker_forces_straight_until_clear_is_confirmed(void)
{
    route_marker_detector_t detector;
    route_marker_detector_output_t output;
    route_marker_detector_config_t config = lap_config();
    line_snapshot_t line = sample(UINT8_C(0x07), 0U);

    assert(route_marker_detector_init(&detector, &config, 0U));
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(output.timestamp_ms == 0U);
    assert(output.marker_present);
    assert(output.force_straight);
    assert(!output.start_cleared);
    assert(!output.start_cleared_event);

    line = sample(UINT8_C(0x08), 1000U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(!output.start_cleared);
    line = sample(UINT8_C(0x08), 1119U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(!output.start_cleared);
    line = sample(UINT8_C(0x08), 1120U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(output.start_cleared);
    assert(output.start_cleared_event);
    assert(!output.force_straight);

    line = sample(UINT8_C(0x08), 1130U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(output.start_cleared);
    assert(!output.start_cleared_event);
}

static void test_sparse_wide_pattern_is_not_an_adjacent_marker(void)
{
    route_marker_detector_t detector;
    route_marker_detector_output_t output;
    route_marker_detector_config_t config = lap_config();
    line_snapshot_t sparse = sample(UINT8_C(0x15), 10U);

    assert(route_marker_detector_init(&detector, &config, 0U));
    assert(route_marker_detector_step(&detector, &sparse, &output));
    assert(sparse.active_count == 3U);
    assert(!output.marker_present);
    assert(!output.force_straight);
}

static void test_marker_confirmation_honors_minimum_run_and_debounce(void)
{
    route_marker_detector_t detector;
    route_marker_detector_output_t output;
    route_marker_detector_config_t config = lap_config();
    line_snapshot_t line;

    assert(route_marker_detector_init(&detector, &config, 0U));
    line = sample(UINT8_C(0x08), 100U);
    assert(route_marker_detector_step(&detector, &line, &output));
    line = sample(UINT8_C(0x08), 220U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(output.start_cleared);

    line = sample(UINT8_C(0x07), 17999U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(output.marker_present);
    assert(!output.marker_confirmed);

    line = sample(UINT8_C(0x07), 18000U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(!output.marker_confirmed);
    line = sample(UINT8_C(0x08), 18010U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(!output.marker_confirmed);
    line = sample(UINT8_C(0x07), 18020U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(!output.marker_confirmed);
    line = sample(UINT8_C(0x07), 18039U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(!output.marker_confirmed);
    line = sample(UINT8_C(0x07), 18040U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(output.marker_confirmed);
    assert(output.marker_confirmed_event);

    line = sample(UINT8_C(0x07), 18050U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(output.marker_confirmed);
    assert(!output.marker_confirmed_event);
}

static void test_optional_adjacency_supports_generic_marker_facts(void)
{
    route_marker_detector_t detector;
    route_marker_detector_output_t output;
    route_marker_detector_config_t config = lap_config();
    line_snapshot_t line;

    config.marker_adjacent_width = 0U;
    config.marker_min_elapsed_ms = 0U;
    config.marker_confirm_ms = 0U;
    assert(route_marker_detector_init(&detector, &config, 0U));

    line = sample(UINT8_C(0x08), 10U);
    assert(route_marker_detector_step(&detector, &line, &output));
    line = sample(UINT8_C(0x08), 130U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(output.start_cleared);

    line = sample(UINT8_C(0x15), 140U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(output.marker_present);
    assert(output.marker_confirmed);
    assert(output.marker_confirmed_event);
}

static void test_clear_timer_resets_and_survives_tick_wrap(void)
{
    route_marker_detector_t detector;
    route_marker_detector_output_t output;
    route_marker_detector_config_t config = lap_config();
    line_snapshot_t line;

    assert(route_marker_detector_init(
        &detector, &config, UINT32_MAX - 100U));
    line = sample(UINT8_C(0x08), UINT32_MAX - 50U);
    assert(route_marker_detector_step(&detector, &line, &output));
    line = sample(0U, UINT32_MAX - 1U);
    assert(route_marker_detector_step(&detector, &line, &output));
    line = sample(UINT8_C(0x08), 0U);
    assert(route_marker_detector_step(&detector, &line, &output));
    line = sample(UINT8_C(0x08), 119U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(!output.start_cleared);
    line = sample(UINT8_C(0x08), 120U);
    assert(route_marker_detector_step(&detector, &line, &output));
    assert(output.start_cleared);
}

static void test_invalid_arguments_fail_without_side_effects(void)
{
    route_marker_detector_t detector;
    route_marker_detector_output_t output;
    route_marker_detector_config_t config = lap_config();
    line_snapshot_t line = sample(UINT8_C(0x08), 10U);

    config.marker_active_threshold = 0U;
    assert(!route_marker_detector_init(&detector, &config, 0U));
    config = lap_config();
    config.marker_adjacent_width = 9U;
    assert(!route_marker_detector_init(&detector, &config, 0U));
    config = lap_config();
    assert(!route_marker_detector_init(NULL, &config, 0U));
    assert(!route_marker_detector_init(&detector, NULL, 0U));
    assert(route_marker_detector_init(&detector, &config, 0U));
    assert(!route_marker_detector_step(NULL, &line, &output));
    assert(!route_marker_detector_step(&detector, NULL, &output));
    assert(!route_marker_detector_step(&detector, &line, NULL));
}

int main(void)
{
    test_start_marker_forces_straight_until_clear_is_confirmed();
    test_sparse_wide_pattern_is_not_an_adjacent_marker();
    test_marker_confirmation_honors_minimum_run_and_debounce();
    test_optional_adjacency_supports_generic_marker_facts();
    test_clear_timer_resets_and_survives_tick_wrap();
    test_invalid_arguments_fail_without_side_effects();
    return 0;
}

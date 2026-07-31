#include "line_snapshot.h"

#include <assert.h>
#include <stdint.h>

static uint8_t raw_for_line_mask(uint8_t line_mask)
{
    return (uint8_t)(~line_mask);
}

static void test_no_active_sensor_is_invalid(void)
{
    line_snapshot_t snapshot = line_snapshot_decode(0xFFU, 1234U);

    assert(!snapshot.valid);
    assert(snapshot.timestamp_ms == 1234U);
    assert(snapshot.raw == 0xFFU);
    assert(snapshot.line_mask == 0U);
    assert(snapshot.active_count == 0U);
    assert(snapshot.weighted_sum == 0);
    assert(snapshot.weighted_error == 0);
}

static void test_each_single_sensor_uses_frozen_q2_weight(void)
{
    static const int16_t expected_weights[8] = {
        -35, -25, -15, -5, 5, 15, 25, 35
    };
    uint8_t index;

    for (index = 0U; index < 8U; index++)
    {
        uint8_t line_mask = (uint8_t)(1U << index);
        line_snapshot_t snapshot =
            line_snapshot_decode(raw_for_line_mask(line_mask), index);

        assert(snapshot.valid);
        assert(snapshot.line_mask == line_mask);
        assert(snapshot.active_count == 1U);
        assert(snapshot.weighted_sum == expected_weights[index]);
        assert(snapshot.weighted_error == expected_weights[index]);
    }
}

static void test_center_channels_keep_minus_five_and_plus_five(void)
{
    line_snapshot_t left =
        line_snapshot_decode(raw_for_line_mask(0x08U), 10U);
    line_snapshot_t right =
        line_snapshot_decode(raw_for_line_mask(0x10U), 11U);

    assert(left.weighted_error == -5);
    assert(right.weighted_error == 5);
}

static void test_multiple_active_sensors_use_integer_average(void)
{
    line_snapshot_t symmetric =
        line_snapshot_decode(raw_for_line_mask(0x18U), 20U);
    line_snapshot_t asymmetric =
        line_snapshot_decode(raw_for_line_mask(0x07U), 21U);

    assert(symmetric.active_count == 2U);
    assert(symmetric.weighted_sum == 0);
    assert(symmetric.weighted_error == 0);
    assert(asymmetric.active_count == 3U);
    assert(asymmetric.weighted_sum == -75);
    assert(asymmetric.weighted_error == -25);
}

static void test_adjacent_width_detection_does_not_accept_sparse_patterns(void)
{
    line_snapshot_t three =
        line_snapshot_decode(raw_for_line_mask(0x1CU), 30U);
    line_snapshot_t four =
        line_snapshot_decode(raw_for_line_mask(0x78U), 31U);
    line_snapshot_t sparse =
        line_snapshot_decode(raw_for_line_mask(0x55U), 32U);

    assert(line_snapshot_has_adjacent(&three, 3U));
    assert(!line_snapshot_has_adjacent(&three, 4U));
    assert(line_snapshot_has_adjacent(&four, 3U));
    assert(line_snapshot_has_adjacent(&four, 4U));
    assert(!line_snapshot_has_adjacent(&sparse, 2U));
    assert(!line_snapshot_has_adjacent(&sparse, 3U));
    assert(!line_snapshot_has_adjacent(NULL, 3U));
    assert(!line_snapshot_has_adjacent(&four, 0U));
    assert(!line_snapshot_has_adjacent(&four, 9U));
}

static void test_timestamp_is_preserved_exactly(void)
{
    line_snapshot_t snapshot =
        line_snapshot_decode(raw_for_line_mask(0x10U), UINT32_MAX);

    assert(snapshot.timestamp_ms == UINT32_MAX);
}

static void test_all_raw_values_match_legacy_q2_decode(void)
{
    static const int8_t weights[8] = {
        -35, -25, -15, -5, 5, 15, 25, 35
    };
    uint16_t raw_value;

    for (raw_value = 0U; raw_value <= UINT8_MAX; raw_value++)
    {
        const uint8_t raw = (uint8_t)raw_value;
        const uint8_t legacy_mask = (uint8_t)(~raw);
        uint8_t legacy_count = 0U;
        int16_t legacy_sum = 0;
        int16_t legacy_error = 0;
        uint8_t index;
        line_snapshot_t snapshot;

        for (index = 0U; index < 8U; index++)
        {
            if ((legacy_mask & (uint8_t)(1U << index)) != 0U)
            {
                legacy_count++;
                legacy_sum += weights[index];
            }
        }
        if (legacy_count != 0U)
        {
            legacy_error = legacy_sum / (int16_t)legacy_count;
        }

        snapshot = line_snapshot_decode(raw, (uint32_t)raw_value);
        assert(snapshot.valid == (legacy_count != 0U));
        assert(snapshot.raw == raw);
        assert(snapshot.line_mask == legacy_mask);
        assert(snapshot.active_count == legacy_count);
        assert(snapshot.weighted_sum == legacy_sum);
        assert(snapshot.weighted_error == legacy_error);
    }
}

int main(void)
{
    test_no_active_sensor_is_invalid();
    test_each_single_sensor_uses_frozen_q2_weight();
    test_center_channels_keep_minus_five_and_plus_five();
    test_multiple_active_sensors_use_integer_average();
    test_adjacent_width_detection_does_not_accept_sparse_patterns();
    test_timestamp_is_preserved_exactly();
    test_all_raw_values_match_legacy_q2_decode();
    return 0;
}

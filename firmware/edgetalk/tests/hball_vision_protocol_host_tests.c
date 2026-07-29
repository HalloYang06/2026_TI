#include "hball_vision_protocol.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static const uint8_t golden_frame[HBALL_VISION_FRAME_SIZE] = {
    0xa5, 0x5a, 0x01, 0x01, 0x40, 0x00, 0x33, 0x00,
    0x40, 0x30, 0x20, 0x10, 0x08, 0x07, 0x06, 0x05,
    0x04, 0x03, 0x02, 0x01, 0xc4, 0x09, 0x00, 0x00,
    0x00, 0x80, 0xf6, 0x42, 0x00, 0x00, 0x36, 0x42,
    0x00, 0x00, 0xd8, 0x40, 0x7b, 0x14, 0x2e, 0xbd,
    0x00, 0x00, 0x70, 0x3f, 0x00, 0x00, 0x0f, 0x43,
    0x10, 0x00, 0x14, 0x00, 0x40, 0x01, 0x78, 0x00,
    0x8d, 0x20, 0x00, 0x00, 0xd3, 0x19, 0x52, 0xdc
};

typedef struct
{
    uint32_t count;
    uint32_t last_sequence;
} capture_t;

static void capture_measurement(
    const hball_vision_measurement_t *measurement, void *context
)
{
    capture_t *capture = (capture_t *)context;

    capture->count++;
    capture->last_sequence = measurement->sequence;
}

static void test_crc32c_and_golden_frame(void)
{
    hball_vision_measurement_t measurement;

    assert(hball_vision_crc32c((const uint8_t *)"123456789", 9U)
        == UINT32_C(0xe3069283));
    assert(hball_vision_decode(golden_frame, sizeof(golden_frame), &measurement)
        == HBALL_VISION_DECODE_OK);
    assert(measurement.sequence == UINT32_C(0x10203040));
    assert(measurement.capture_time_us == UINT64_C(0x0102030405060708));
    assert(measurement.processing_time_us == 2500U);
    assert(measurement.center_x_px == 123.25F);
    assert(measurement.center_y_px == 45.5F);
    assert(measurement.radius_px == 6.75F);
    assert(fabsf(measurement.ball_position_m - (-0.0425F)) < 1.0e-7F);
    assert(measurement.confidence == 0.9375F);
    assert(measurement.contour_area_px2 == 143.0F);
    assert(measurement.roi_x == 16U && measurement.roi_y == 20U);
    assert(measurement.roi_w == 320U && measurement.roi_h == 120U);
    assert(measurement.exposure_us == 8333U);
}

static void test_crc_damage_is_rejected(void)
{
    hball_vision_measurement_t measurement;
    uint8_t damaged[HBALL_VISION_FRAME_SIZE];

    memcpy(damaged, golden_frame, sizeof(damaged));
    damaged[32] ^= 0x01U;
    assert(hball_vision_decode(damaged, sizeof(damaged), &measurement)
        == HBALL_VISION_DECODE_CRC);
}

static void test_stream_resynchronizes_across_usb_chunks(void)
{
    hball_vision_stream_t stream;
    capture_t capture = {0U, 0U};
    uint8_t damaged[HBALL_VISION_FRAME_SIZE];
    const uint8_t noise[] = {'t', 't', 'y', '-', 'n', 'o', 'i', 's', 'e'};

    memcpy(damaged, golden_frame, sizeof(damaged));
    damaged[24] ^= 0x80U;
    hball_vision_stream_init(&stream);
    assert(hball_vision_stream_push(
        &stream, noise, sizeof(noise), capture_measurement, &capture
    ) == 0U);
    assert(hball_vision_stream_push(
        &stream, damaged, 37U, capture_measurement, &capture
    ) == 0U);
    assert(hball_vision_stream_push(
        &stream, damaged + 37U, sizeof(damaged) - 37U,
        capture_measurement, &capture
    ) == 0U);
    assert(hball_vision_stream_push(
        &stream, golden_frame, 11U, capture_measurement, &capture
    ) == 0U);
    assert(hball_vision_stream_push(
        &stream, golden_frame + 11U, sizeof(golden_frame) - 11U,
        capture_measurement, &capture
    ) == 1U);
    assert(capture.count == 1U);
    assert(capture.last_sequence == UINT32_C(0x10203040));
    assert(stream.crc_failure_total == 1U);
    assert(stream.accepted_total == 1U);
}

int main(void)
{
    test_crc32c_and_golden_frame();
    test_crc_damage_is_rejected();
    test_stream_resynchronizes_across_usb_chunks();
    return 0;
}

#include "hball_vision_protocol.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

_Static_assert(sizeof(float) == 4U, "vision protocol requires IEEE-754 float32");

static uint16_t hball_load_u16(const uint8_t *source)
{
    return (uint16_t)((uint16_t)source[0]
        | ((uint16_t)source[1] << 8U));
}

static uint32_t hball_load_u32(const uint8_t *source)
{
    return (uint32_t)source[0]
        | ((uint32_t)source[1] << 8U)
        | ((uint32_t)source[2] << 16U)
        | ((uint32_t)source[3] << 24U);
}

static uint64_t hball_load_u64(const uint8_t *source)
{
    return (uint64_t)hball_load_u32(source)
        | ((uint64_t)hball_load_u32(source + 4U) << 32U);
}

static float hball_load_float(const uint8_t *source)
{
    const uint32_t bits = hball_load_u32(source);
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

uint32_t hball_vision_crc32c(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_C(0xffffffff);

    if ((data == NULL) && (length != 0U))
    {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (unsigned bit = 0U; bit < 8U; ++bit)
        {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);

            crc = (crc >> 1U) ^ (UINT32_C(0x82f63b78) & mask);
        }
    }
    return crc ^ UINT32_C(0xffffffff);
}

static bool hball_measurement_is_valid(
    const hball_vision_measurement_t *measurement
)
{
    return isfinite(measurement->center_x_px)
        && isfinite(measurement->center_y_px)
        && isfinite(measurement->radius_px)
        && isfinite(measurement->ball_position_m)
        && isfinite(measurement->confidence)
        && isfinite(measurement->contour_area_px2)
        && (measurement->radius_px >= 0.0F)
        && (measurement->confidence >= 0.0F)
        && (measurement->confidence <= 1.0F)
        && (measurement->contour_area_px2 >= 0.0F);
}

hball_vision_decode_result_t hball_vision_decode(
    const uint8_t *frame,
    size_t length,
    hball_vision_measurement_t *measurement
)
{
    if ((frame == NULL) || (measurement == NULL)
        || (length != HBALL_VISION_FRAME_SIZE))
    {
        return HBALL_VISION_DECODE_ARGUMENT;
    }
    if (hball_load_u16(frame) != HBALL_VISION_MAGIC)
    {
        return HBALL_VISION_DECODE_MAGIC;
    }
    if ((frame[2] != HBALL_VISION_VERSION)
        || (frame[3] != HBALL_VISION_MESSAGE_TYPE)
        || (hball_load_u16(frame + 4U) != HBALL_VISION_FRAME_SIZE))
    {
        return HBALL_VISION_DECODE_HEADER;
    }
    if (hball_vision_crc32c(frame, HBALL_VISION_FRAME_SIZE - 4U)
        != hball_load_u32(frame + HBALL_VISION_FRAME_SIZE - 4U))
    {
        return HBALL_VISION_DECODE_CRC;
    }

    measurement->flags = hball_load_u16(frame + 6U);
    measurement->sequence = hball_load_u32(frame + 8U);
    measurement->capture_time_us = hball_load_u64(frame + 12U);
    measurement->processing_time_us = hball_load_u32(frame + 20U);
    measurement->center_x_px = hball_load_float(frame + 24U);
    measurement->center_y_px = hball_load_float(frame + 28U);
    measurement->radius_px = hball_load_float(frame + 32U);
    measurement->ball_position_m = hball_load_float(frame + 36U);
    measurement->confidence = hball_load_float(frame + 40U);
    measurement->contour_area_px2 = hball_load_float(frame + 44U);
    measurement->roi_x = hball_load_u16(frame + 48U);
    measurement->roi_y = hball_load_u16(frame + 50U);
    measurement->roi_w = hball_load_u16(frame + 52U);
    measurement->roi_h = hball_load_u16(frame + 54U);
    measurement->exposure_us = hball_load_u32(frame + 56U);

    return hball_measurement_is_valid(measurement)
        ? HBALL_VISION_DECODE_OK
        : HBALL_VISION_DECODE_RANGE;
}

void hball_vision_stream_init(hball_vision_stream_t *stream)
{
    if (stream != NULL)
    {
        memset(stream, 0, sizeof(*stream));
    }
}

static void hball_vision_stream_resync(hball_vision_stream_t *stream)
{
    size_t keep_at = stream->length;

    for (size_t index = 1U; (index + 1U) < stream->length; ++index)
    {
        if ((stream->buffer[index] == 0xa5U)
            && (stream->buffer[index + 1U] == 0x5aU))
        {
            keep_at = index;
            break;
        }
    }
    if ((keep_at == stream->length) && (stream->length > 0U)
        && (stream->buffer[stream->length - 1U] == 0xa5U))
    {
        keep_at = stream->length - 1U;
    }
    stream->discarded_byte_total += (uint32_t)keep_at;
    stream->length -= keep_at;
    if (stream->length > 0U)
    {
        memmove(stream->buffer, stream->buffer + keep_at, stream->length);
    }
}

static void hball_vision_stream_count_failure(
    hball_vision_stream_t *stream, hball_vision_decode_result_t result
)
{
    if (result == HBALL_VISION_DECODE_CRC)
    {
        stream->crc_failure_total++;
    }
    else if (result == HBALL_VISION_DECODE_RANGE)
    {
        stream->range_failure_total++;
    }
    else
    {
        stream->header_failure_total++;
    }
}

size_t hball_vision_stream_push(
    hball_vision_stream_t *stream,
    const uint8_t *data,
    size_t length,
    hball_vision_measurement_handler_t handler,
    void *context
)
{
    size_t accepted = 0U;

    if ((stream == NULL) || ((data == NULL) && (length != 0U)))
    {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index)
    {
        const uint8_t value = data[index];

        if ((stream->length == 0U) && (value != 0xa5U))
        {
            stream->discarded_byte_total++;
            continue;
        }
        if ((stream->length == 1U) && (value != 0x5aU))
        {
            if (value == 0xa5U)
            {
                stream->discarded_byte_total++;
            }
            else
            {
                stream->discarded_byte_total += 2U;
                stream->length = 0U;
            }
            continue;
        }
        stream->buffer[stream->length++] = value;
        if (stream->length == HBALL_VISION_FRAME_SIZE)
        {
            hball_vision_measurement_t measurement;
            const hball_vision_decode_result_t result = hball_vision_decode(
                stream->buffer, stream->length, &measurement
            );

            if (result == HBALL_VISION_DECODE_OK)
            {
                stream->accepted_total++;
                accepted++;
                stream->length = 0U;
                if (handler != NULL)
                {
                    handler(&measurement, context);
                }
            }
            else
            {
                hball_vision_stream_count_failure(stream, result);
                hball_vision_stream_resync(stream);
            }
        }
    }
    return accepted;
}

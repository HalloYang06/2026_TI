#ifndef HBALL_VISION_PROTOCOL_H
#define HBALL_VISION_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_VISION_MAGIC UINT16_C(0x5aa5)
#define HBALL_VISION_VERSION 1U
#define HBALL_VISION_MESSAGE_TYPE 1U
#define HBALL_VISION_FRAME_SIZE 64U

#define HBALL_VISION_FLAG_DETECTED (UINT16_C(1) << 0)
#define HBALL_VISION_FLAG_POSITION_VALID (UINT16_C(1) << 1)
#define HBALL_VISION_FLAG_ROI_PREDICTED (UINT16_C(1) << 2)
#define HBALL_VISION_FLAG_ROI_CLIPPED (UINT16_C(1) << 3)
#define HBALL_VISION_FLAG_EXPOSURE_STABLE (UINT16_C(1) << 4)
#define HBALL_VISION_FLAG_CONTOUR_ROUND (UINT16_C(1) << 5)

typedef struct
{
    uint16_t flags;
    uint32_t sequence;
    uint64_t capture_time_us;
    uint32_t processing_time_us;
    float center_x_px;
    float center_y_px;
    float radius_px;
    float ball_position_m;
    float confidence;
    float contour_area_px2;
    uint16_t roi_x;
    uint16_t roi_y;
    uint16_t roi_w;
    uint16_t roi_h;
    uint32_t exposure_us;
} hball_vision_measurement_t;

typedef enum
{
    HBALL_VISION_DECODE_OK = 0,
    HBALL_VISION_DECODE_ARGUMENT,
    HBALL_VISION_DECODE_MAGIC,
    HBALL_VISION_DECODE_HEADER,
    HBALL_VISION_DECODE_CRC,
    HBALL_VISION_DECODE_RANGE
} hball_vision_decode_result_t;

typedef struct
{
    uint8_t buffer[HBALL_VISION_FRAME_SIZE];
    size_t length;
    uint32_t accepted_total;
    uint32_t crc_failure_total;
    uint32_t header_failure_total;
    uint32_t range_failure_total;
    uint32_t discarded_byte_total;
} hball_vision_stream_t;

typedef void (*hball_vision_measurement_handler_t)(
    const hball_vision_measurement_t *measurement, void *context
);

uint32_t hball_vision_crc32c(const uint8_t *data, size_t length);
hball_vision_decode_result_t hball_vision_decode(
    const uint8_t *frame,
    size_t length,
    hball_vision_measurement_t *measurement
);
void hball_vision_stream_init(hball_vision_stream_t *stream);
size_t hball_vision_stream_push(
    hball_vision_stream_t *stream,
    const uint8_t *data,
    size_t length,
    hball_vision_measurement_handler_t handler,
    void *context
);

#ifdef __cplusplus
}
#endif

#endif

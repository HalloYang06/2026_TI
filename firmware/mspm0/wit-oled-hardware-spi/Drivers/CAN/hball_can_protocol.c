#include "hball_can_protocol.h"

#include <limits.h>
#include <stddef.h>
#include <string.h>

static int16_t hball_saturate_i16(int32_t value)
{
    if (value > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (value < INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)value;
}

static int32_t hball_round_float(float value)
{
    return (int32_t)(value + ((value >= 0.0F) ? 0.5F : -0.5F));
}

static void hball_store_u16_le(uint8_t *target, uint16_t value)
{
    target[0] = (uint8_t)value;
    target[1] = (uint8_t)(value >> 8U);
}

static void hball_store_u32_le(uint8_t *target, uint32_t value)
{
    target[0] = (uint8_t)value;
    target[1] = (uint8_t)(value >> 8U);
    target[2] = (uint8_t)(value >> 16U);
    target[3] = (uint8_t)(value >> 24U);
}

static void hball_store_vector(
    uint8_t *target, const int16_t values[3]
)
{
    hball_store_u16_le(target, (uint16_t)values[0]);
    hball_store_u16_le(target + 2U, (uint16_t)values[1]);
    hball_store_u16_le(target + 4U, (uint16_t)values[2]);
}

bool hball_can_stream_due(uint32_t now_ms, hball_can_stream_t *stream)
{
    if (stream == NULL)
    {
        return false;
    }

    switch (now_ms % 10U)
    {
    case 0U:
    case 5U:
        *stream = HBALL_CAN_STREAM_ACCEL;
        return true;
    case 1U:
    case 6U:
        *stream = HBALL_CAN_STREAM_GYRO;
        return true;
    case 2U:
        *stream = HBALL_CAN_STREAM_WHEEL;
        return true;
    case 3U:
    case 8U:
        *stream = HBALL_CAN_STREAM_ATTITUDE;
        return true;
    default:
        break;
    }

    if ((now_ms % 50U) == 4U)
    {
        *stream = HBALL_CAN_STREAM_HEARTBEAT;
        return true;
    }
    return false;
}

bool hball_can_encode_frame(
    hball_can_stream_t stream,
    uint16_t sequence,
    const hball_can_inputs_t *inputs,
    hball_can_frame_t *frame
)
{
    int16_t wheel_values[3];

    if ((stream >= HBALL_CAN_STREAM_COUNT)
        || (inputs == NULL)
        || (frame == NULL))
    {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->dlc = 8U;
    hball_store_u16_le(frame->data, sequence);

    switch (stream)
    {
    case HBALL_CAN_STREAM_ACCEL:
        frame->id = HBALL_MSP_CAN_ID_ACCEL;
        hball_store_vector(frame->data + 2U, inputs->accel_milli_mps2);
        break;
    case HBALL_CAN_STREAM_GYRO:
        frame->id = HBALL_MSP_CAN_ID_GYRO;
        hball_store_vector(frame->data + 2U, inputs->gyro_milli_rad_s);
        break;
    case HBALL_CAN_STREAM_WHEEL:
        frame->id = HBALL_MSP_CAN_ID_WHEEL;
        wheel_values[0] = inputs->wheel_milli_mps[0];
        wheel_values[1] = inputs->wheel_milli_mps[1];
        wheel_values[2] = inputs->body_milli_mps;
        hball_store_vector(frame->data + 2U, wheel_values);
        break;
    case HBALL_CAN_STREAM_ATTITUDE:
        frame->id = HBALL_MSP_CAN_ID_ATTITUDE;
        hball_store_vector(frame->data + 2U, inputs->attitude_milli_rad);
        break;
    case HBALL_CAN_STREAM_HEARTBEAT:
        frame->id = HBALL_MSP_CAN_ID_HEARTBEAT;
        hball_store_u16_le(frame->data + 2U, inputs->status_flags);
        hball_store_u32_le(frame->data + 4U, inputs->uptime_ms);
        break;
    default:
        return false;
    }

    return true;
}

int16_t hball_can_mg_to_milli_mps2(int16_t acceleration_mg)
{
    int64_t scaled = (int64_t)acceleration_mg * INT64_C(980665);

    scaled += (scaled >= 0) ? INT64_C(50000) : -INT64_C(50000);
    scaled /= INT64_C(100000);
    if (scaled > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (scaled < INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)scaled;
}

int16_t hball_can_dps_to_milli_rad_s(int16_t angular_velocity_dps)
{
    return hball_saturate_i16(hball_round_float(
        (float)angular_velocity_dps * 17.45329252F));
}

int16_t hball_can_deg_to_milli_rad(float angle_deg)
{
    return hball_saturate_i16(hball_round_float(angle_deg * 17.45329252F));
}

int16_t hball_can_counts_per_20ms_to_milli_mps(
    int32_t counts,
    uint32_t wheel_circumference_mm,
    uint32_t counts_per_revolution
)
{
    int64_t scaled;

    if ((wheel_circumference_mm == 0U) || (counts_per_revolution == 0U))
    {
        return 0;
    }
    scaled = (int64_t)counts * (int64_t)wheel_circumference_mm * 50;
    scaled /= (int64_t)counts_per_revolution;
    if (scaled > INT16_MAX)
    {
        return INT16_MAX;
    }
    if (scaled < INT16_MIN)
    {
        return INT16_MIN;
    }
    return (int16_t)scaled;
}

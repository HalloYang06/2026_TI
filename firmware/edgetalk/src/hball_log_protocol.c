#include "hball_log_protocol.h"

#include <math.h>
#include <string.h>

static void put_u16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static void put_float(uint8_t *destination, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    put_u32(destination, bits);
}

uint32_t hball_log_crc32c(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_MAX;

    if (data == NULL)
    {
        return 0U;
    }
    for (size_t index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (unsigned bit = 0U; bit < 8U; ++bit)
        {
            crc = (crc >> 1) ^ ((crc & 1U) != 0U
                ? UINT32_C(0x82F63B78) : 0U);
        }
    }
    return ~crc;
}

bool hball_log_encode(
    const hball_log_record_t *record,
    uint8_t frame[HBALL_LOG_FRAME_SIZE]
)
{
    const float values[] = {
        record != NULL ? record->ball_position_m : 0.0F,
        record != NULL ? record->estimated_position_m : 0.0F,
        record != NULL ? record->estimated_velocity_mps : 0.0F,
        record != NULL ? record->estimated_disturbance_mps2 : 0.0F,
        record != NULL ? record->pipe_target_rad : 0.0F,
        record != NULL ? record->motor_angle_rad : 0.0F,
        record != NULL ? record->motor_velocity_rad_s : 0.0F,
        record != NULL ? record->longitudinal_accel_mps2 : 0.0F,
        record != NULL ? record->body_pitch_rad : 0.0F,
    };

    if ((record == NULL) || (frame == NULL))
    {
        return false;
    }
    for (size_t index = 0U; index < sizeof(values) / sizeof(values[0]); ++index)
    {
        if (!isfinite(values[index]))
        {
            return false;
        }
    }
    memset(frame, 0, HBALL_LOG_FRAME_SIZE);
    put_u32(frame + 0U, HBALL_LOG_MAGIC);
    put_u16(frame + 4U, HBALL_LOG_VERSION);
    put_u16(frame + 6U, HBALL_LOG_FRAME_SIZE);
    put_u32(frame + 8U, record->sequence);
    put_u32(frame + 12U, record->produced_time_ms);
    put_u32(frame + 16U, record->sensor_sequence);
    put_u32(frame + 20U, record->controller_steps);
    put_u32(frame + 24U, record->vision_sequence);
    put_u32(frame + 28U, record->sensor_valid_flags);
    put_u16(frame + 32U, record->control_mode);
    put_u16(frame + 34U, record->guard_reason);
    put_u32(frame + 36U, record->status_flags);
    for (size_t index = 0U; index < sizeof(values) / sizeof(values[0]); ++index)
    {
        put_float(frame + 40U + index * 4U, values[index]);
    }
    put_u32(frame + 76U, hball_log_crc32c(frame, 76U));
    return true;
}

#include "hball_log_protocol.h"

#include <assert.h>
#include <string.h>

static uint16_t load_u16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t load_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U)
        | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

int main(void)
{
    hball_log_record_t record;
    uint8_t frame[HBALL_LOG_FRAME_SIZE];

    memset(&record, 0, sizeof(record));
    record.sequence = 7U;
    record.produced_time_ms = 1234U;
    record.vision_capture_time_us = UINT64_C(0x0102030405060708);
    record.imu_source_time_ms = 1200U;
    record.ball_position_m = -0.015F;
    record.pipe_target_rad = 0.02F;
    record.accel_mps2[2] = 9.80665F;
    assert(hball_log_encode(&record, frame));
    assert(memcmp(frame, "HBLG", 4U) == 0);
    assert(load_u16(frame + 4U) == HBALL_LOG_VERSION);
    assert(load_u16(frame + 6U) == HBALL_LOG_FRAME_SIZE);
    assert(load_u32(frame + 60U) == 1200U);
    assert(hball_log_crc32c(frame, HBALL_LOG_FRAME_SIZE - 4U)
        == load_u32(frame + HBALL_LOG_FRAME_SIZE - 4U));
    record.attitude_rad[1] = 0.0F / 0.0F;
    assert(!hball_log_encode(&record, frame));
    return 0;
}

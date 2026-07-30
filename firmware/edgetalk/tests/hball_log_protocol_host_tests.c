#include "hball_log_protocol.h"

#include <assert.h>
#include <string.h>

int main(void)
{
    hball_log_record_t record;
    uint8_t frame[HBALL_LOG_FRAME_SIZE];

    memset(&record, 0, sizeof(record));
    record.sequence = 7U;
    record.produced_time_ms = 1234U;
    record.ball_position_m = -0.015F;
    record.pipe_target_rad = 0.02F;
    assert(hball_log_encode(&record, frame));
    assert(memcmp(frame, "HBLG", 4U) == 0);
    assert(frame[6] == HBALL_LOG_FRAME_SIZE);
    assert(hball_log_crc32c(frame, 76U)
        == ((uint32_t)frame[76] | ((uint32_t)frame[77] << 8)
            | ((uint32_t)frame[78] << 16) | ((uint32_t)frame[79] << 24)));
    record.body_pitch_rad = 0.0F / 0.0F;
    assert(!hball_log_encode(&record, frame));
    return 0;
}

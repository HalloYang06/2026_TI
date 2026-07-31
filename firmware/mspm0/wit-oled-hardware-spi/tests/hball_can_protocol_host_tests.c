#include "hball_can_protocol.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

static uint16_t load_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t load_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U)
        | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

static void test_scheduler_meets_the_six_stream_rates_without_collisions(void)
{
    uint32_t counts[HBALL_CAN_STREAM_COUNT] = {0U};

    for (uint32_t now_ms = 0U; now_ms < 1000U; ++now_ms)
    {
        hball_can_stream_t stream = HBALL_CAN_STREAM_COUNT;
        if (hball_can_stream_due(now_ms, &stream))
        {
            assert(stream < HBALL_CAN_STREAM_COUNT);
            counts[stream]++;
        }
    }

    assert(counts[HBALL_CAN_STREAM_ACCEL] == 200U);
    assert(counts[HBALL_CAN_STREAM_GYRO] == 200U);
    assert(counts[HBALL_CAN_STREAM_WHEEL] == 100U);
    assert(counts[HBALL_CAN_STREAM_ATTITUDE] == 200U);
    assert(counts[HBALL_CAN_STREAM_IMU_TIME] == 80U);
    assert(counts[HBALL_CAN_STREAM_HEARTBEAT] == 20U);
}

static void test_imu_time_frame_preserves_source_epoch_and_time(void)
{
    hball_can_inputs_t inputs;
    hball_can_frame_t frame;

    memset(&inputs, 0, sizeof(inputs));
    inputs.imu_epoch = UINT16_C(0x1234);
    inputs.imu_sample_mask = HBALL_MSP_IMU_SAMPLE_ACCEL
        | HBALL_MSP_IMU_SAMPLE_GYRO
        | HBALL_MSP_IMU_SAMPLE_ATTITUDE
        | HBALL_MSP_IMU_SAMPLE_COMPLETE;
    inputs.imu_source_time_ms = UINT32_C(0x89abcdef);
    assert(hball_can_encode_frame(
        HBALL_CAN_STREAM_IMU_TIME, 0U, &inputs, &frame));
    assert(frame.id == HBALL_MSP_CAN_ID_IMU_TIME);
    assert(load_u16_le(frame.data) == inputs.imu_epoch);
    assert(load_u16_le(frame.data + 2U) == inputs.imu_sample_mask);
    assert(load_u32_le(frame.data + 4U) == inputs.imu_source_time_ms);
}

static void test_heartbeat_is_little_endian_classic_standard_can(void)
{
    hball_can_inputs_t inputs;
    hball_can_frame_t frame;

    memset(&inputs, 0, sizeof(inputs));
    inputs.status_flags = HBALL_MSP_STATUS_ESTOP_ACTIVE
        | HBALL_MSP_STATUS_IMU_VALID;
    inputs.uptime_ms = UINT32_C(0x12345678);

    assert(hball_can_encode_frame(
        HBALL_CAN_STREAM_HEARTBEAT, UINT16_C(0xffff), &inputs, &frame));
    assert(frame.id == HBALL_MSP_CAN_ID_HEARTBEAT);
    assert(frame.is_extended == 0U);
    assert(frame.is_remote == 0U);
    assert(frame.dlc == 8U);
    assert(frame.fdf == 0U);
    assert(frame.brs == 0U);
    assert(load_u16_le(frame.data) == UINT16_C(0xffff));
    assert(load_u16_le(frame.data + 2U) == inputs.status_flags);
    assert(load_u32_le(frame.data + 4U) == inputs.uptime_ms);
}

static void test_sensor_frames_preserve_signed_milli_si_values(void)
{
    hball_can_inputs_t inputs;
    hball_can_frame_t frame;

    memset(&inputs, 0, sizeof(inputs));
    inputs.accel_milli_mps2[0] = 1000;
    inputs.accel_milli_mps2[1] = -250;
    inputs.accel_milli_mps2[2] = 9807;
    inputs.gyro_milli_rad_s[0] = -100;
    inputs.gyro_milli_rad_s[1] = 200;
    inputs.gyro_milli_rad_s[2] = -300;
    inputs.wheel_milli_mps[0] = 100;
    inputs.wheel_milli_mps[1] = -100;
    inputs.body_milli_mps = 0;
    inputs.attitude_milli_rad[0] = 10;
    inputs.attitude_milli_rad[1] = -20;
    inputs.attitude_milli_rad[2] = 300;

    assert(hball_can_encode_frame(
        HBALL_CAN_STREAM_ACCEL, 0x1234U, &inputs, &frame));
    assert(frame.id == HBALL_MSP_CAN_ID_ACCEL);
    assert((int16_t)load_u16_le(frame.data + 2U) == 1000);
    assert((int16_t)load_u16_le(frame.data + 4U) == -250);
    assert((int16_t)load_u16_le(frame.data + 6U) == 9807);

    assert(hball_can_encode_frame(
        HBALL_CAN_STREAM_GYRO, 0x1234U, &inputs, &frame));
    assert(frame.id == HBALL_MSP_CAN_ID_GYRO);
    assert((int16_t)load_u16_le(frame.data + 2U) == -100);
    assert((int16_t)load_u16_le(frame.data + 4U) == 200);
    assert((int16_t)load_u16_le(frame.data + 6U) == -300);

    assert(hball_can_encode_frame(
        HBALL_CAN_STREAM_WHEEL, 9U, &inputs, &frame));
    assert(frame.id == HBALL_MSP_CAN_ID_WHEEL);
    assert((int16_t)load_u16_le(frame.data + 2U) == 100);
    assert((int16_t)load_u16_le(frame.data + 4U) == -100);
    assert((int16_t)load_u16_le(frame.data + 6U) == 0);

    assert(hball_can_encode_frame(
        HBALL_CAN_STREAM_ATTITUDE, 0x5678U, &inputs, &frame));
    assert(frame.id == HBALL_MSP_CAN_ID_ATTITUDE);
    assert((int16_t)load_u16_le(frame.data + 2U) == 10);
    assert((int16_t)load_u16_le(frame.data + 4U) == -20);
    assert((int16_t)load_u16_le(frame.data + 6U) == 300);
}

static void test_wit_unit_conversion_rounds_and_saturates(void)
{
    assert(hball_can_mg_to_milli_mps2(1000) == 9807);
    assert(hball_can_dps_to_milli_rad_s(180) == 3142);
    assert(hball_can_deg_to_milli_rad(-90.0F) == -1571);
    assert(hball_can_mg_to_milli_mps2(INT16_MAX) == INT16_MAX);
    assert(hball_can_dps_to_milli_rad_s(INT16_MAX) == INT16_MAX);
}

static void test_wheel_conversion_requires_explicit_mechanical_calibration(void)
{
    assert(hball_can_counts_per_20ms_to_milli_mps(60, 0U, 400U) == 0);
    assert(hball_can_counts_per_20ms_to_milli_mps(60, 210U, 0U) == 0);
    assert(hball_can_counts_per_20ms_to_milli_mps(40, 200U, 400U) == 1000);
    assert(hball_can_counts_per_20ms_to_milli_mps(-40, 200U, 400U) == -1000);
}

static void test_encoder_can_only_emit_the_frozen_read_only_mspm0_ids(void)
{
    static const uint32_t allowed_ids[HBALL_CAN_STREAM_COUNT] = {
        HBALL_MSP_CAN_ID_ACCEL,
        HBALL_MSP_CAN_ID_GYRO,
        HBALL_MSP_CAN_ID_WHEEL,
        HBALL_MSP_CAN_ID_ATTITUDE,
        HBALL_MSP_CAN_ID_IMU_TIME,
        HBALL_MSP_CAN_ID_HEARTBEAT,
    };
    hball_can_inputs_t inputs;
    hball_can_frame_t frame;

    memset(&inputs, 0, sizeof(inputs));
    for (uint32_t stream = 0U; stream < HBALL_CAN_STREAM_COUNT; ++stream)
    {
        assert(hball_can_encode_frame(
            (hball_can_stream_t)stream, 0U, &inputs, &frame));
        assert(frame.id == allowed_ids[stream]);
        assert(frame.id <= UINT32_C(0x7ff));
        assert(frame.is_extended == 0U);
        assert(frame.is_remote == 0U);
        assert(frame.fdf == 0U);
        assert(frame.brs == 0U);
    }
}

int main(void)
{
    test_scheduler_meets_the_six_stream_rates_without_collisions();
    test_imu_time_frame_preserves_source_epoch_and_time();
    test_heartbeat_is_little_endian_classic_standard_can();
    test_sensor_frames_preserve_signed_milli_si_values();
    test_wit_unit_conversion_rounds_and_saturates();
    test_wheel_conversion_requires_explicit_mechanical_calibration();
    test_encoder_can_only_emit_the_frozen_read_only_mspm0_ids();
    return 0;
}

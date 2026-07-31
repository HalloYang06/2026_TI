#include "hball_can.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static void test_get_id_frame_and_probe_reply(void)
{
    hball_motor_monitor_t monitor;
    hball_can_frame_t request;
    hball_can_frame_t reply;
    const uint8_t uid_bytes[8] = {1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};

    hball_motor_monitor_init(&monitor, HBALL_RS00_MOTOR_ID);
    assert(hball_motor_monitor_make_probe(&monitor, &request));
    assert(request.id == 0x0000FD05UL);
    assert(request.is_extended == 1U);
    assert(request.is_remote == 0U);
    assert(request.dlc == 8U);

    memset(&reply, 0, sizeof(reply));
    reply.id = 0x000005FEUL;
    reply.is_extended = 1U;
    reply.dlc = 8U;
    memcpy(reply.data, uid_bytes, sizeof(uid_bytes));

    assert(hball_motor_monitor_accept(&monitor, &reply, 100U) == HBALL_CAN_EVENT_PROBE_REPLY);
    assert(monitor.probe_valid);
    assert(monitor.unique_id == UINT64_C(0x0807060504030201));
    assert(!monitor.probe_pending);
}

static void test_feedback_decode_and_freshness(void)
{
    hball_motor_monitor_t monitor;
    hball_can_frame_t frame;
    const uint16_t data2 = (uint16_t)((2U << 14) | (3U << 8) | HBALL_RS00_MOTOR_ID);

    hball_motor_monitor_init(&monitor, HBALL_RS00_MOTOR_ID);
    memset(&frame, 0, sizeof(frame));
    frame.id = hball_rs00_ext_id(HBALL_RS00_TYPE_FEEDBACK, data2, HBALL_RS00_MASTER_ID);
    frame.is_extended = 1U;
    frame.dlc = 8U;
    frame.data[0] = 0x80U;
    frame.data[1] = 0x00U;
    frame.data[2] = 0x80U;
    frame.data[3] = 0x00U;
    frame.data[4] = 0x80U;
    frame.data[5] = 0x00U;
    frame.data[6] = 0x00U;
    frame.data[7] = 0xFAU;

    assert(hball_motor_monitor_accept(&monitor, &frame, 1000U) == HBALL_CAN_EVENT_FEEDBACK);
    assert(monitor.feedback_valid);
    assert(monitor.feedback.motor_id == HBALL_RS00_MOTOR_ID);
    assert(monitor.feedback.mode_state == 2U);
    assert(monitor.feedback.fault_summary == 3U);
    assert(fabsf(monitor.feedback.position_rad) < 0.001F);
    assert(fabsf(monitor.feedback.velocity_rad_s) < 0.001F);
    assert(fabsf(monitor.feedback.torque_nm) < 0.001F);
    assert(fabsf(monitor.feedback.temperature_c - 25.0F) < 0.001F);
    assert(hball_motor_monitor_feedback_fresh(&monitor, 1049U, 50U));
    assert(!hball_motor_monitor_feedback_fresh(&monitor, 1051U, 50U));
}

static void test_invalid_or_wrong_host_frames_are_not_feedback(void)
{
    hball_motor_monitor_t monitor;
    hball_can_frame_t frame;

    hball_motor_monitor_init(&monitor, HBALL_RS00_MOTOR_ID);
    memset(&frame, 0, sizeof(frame));
    frame.id = hball_rs00_ext_id(HBALL_RS00_TYPE_FEEDBACK, HBALL_RS00_MOTOR_ID, 0xAAU);
    frame.is_extended = 1U;
    frame.dlc = 8U;
    assert(hball_motor_monitor_accept(&monitor, &frame, 1U) == HBALL_CAN_EVENT_IGNORED);

    frame.id = hball_rs00_ext_id(
        HBALL_RS00_TYPE_FEEDBACK, HBALL_RS00_MOTOR_ID, HBALL_RS00_MASTER_ID
    );
    frame.dlc = 4U;
    assert(hball_motor_monitor_accept(&monitor, &frame, 2U) == HBALL_CAN_EVENT_INVALID);
    assert(monitor.rx_total == 2U);
    assert(monitor.rx_invalid == 1U);
}

static void hball_make_parameter_reply(
    hball_can_frame_t *frame, uint16_t index, const uint8_t value[4]
)
{
    memset(frame, 0, sizeof(*frame));
    frame->id = hball_rs00_ext_id(
        HBALL_RS00_TYPE_GET_SINGLE_PARAMETER,
        HBALL_RS00_MOTOR_ID,
        HBALL_RS00_MASTER_ID
    );
    frame->is_extended = 1U;
    frame->dlc = 8U;
    frame->data[0] = (uint8_t)index;
    frame->data[1] = (uint8_t)(index >> 8U);
    memcpy(frame->data + 4U, value, 4U);
}

static void test_parameter_read_request_is_strictly_whitelisted(void)
{
    static const uint16_t indexes[] = {
        HBALL_RS00_INDEX_RUN_MODE,
        HBALL_RS00_INDEX_MECH_POSITION,
        HBALL_RS00_INDEX_FILTERED_IQ,
        HBALL_RS00_INDEX_MECH_VELOCITY,
        HBALL_RS00_INDEX_VBUS,
        HBALL_RS00_INDEX_ROTATION,
    };
    hball_motor_monitor_t monitor;
    hball_can_frame_t request;
    size_t index;

    hball_motor_monitor_init(&monitor, HBALL_RS00_MOTOR_ID);
    assert(!hball_rs00_parameter_is_read_only(0x7016U));
    assert(!hball_motor_monitor_make_parameter_read(
        &monitor, 0x7016U, 1U, &request
    ));

    for (index = 0U; index < sizeof(indexes) / sizeof(indexes[0]); ++index)
    {
        assert(hball_rs00_parameter_is_read_only(indexes[index]));
    }
    assert(hball_motor_monitor_make_parameter_read(
        &monitor, HBALL_RS00_INDEX_MECH_POSITION, 10U, &request
    ));
    assert(request.id == 0x1100FD05UL);
    assert(request.is_extended == 1U);
    assert(request.is_remote == 0U);
    assert(request.dlc == 8U);
    assert(request.data[0] == 0x19U);
    assert(request.data[1] == 0x70U);
    for (index = 2U; index < 8U; ++index)
    {
        assert(request.data[index] == 0U);
    }
    assert(monitor.parameter_pending);
    assert(monitor.pending_parameter_index == HBALL_RS00_INDEX_MECH_POSITION);
    assert(!hball_motor_monitor_make_parameter_read(
        &monitor, HBALL_RS00_INDEX_VBUS, 11U, &request
    ));
}

static void test_parameter_replies_decode_all_required_motor_data(void)
{
    static const uint16_t indexes[] = {
        HBALL_RS00_INDEX_RUN_MODE,
        HBALL_RS00_INDEX_MECH_POSITION,
        HBALL_RS00_INDEX_FILTERED_IQ,
        HBALL_RS00_INDEX_MECH_VELOCITY,
        HBALL_RS00_INDEX_VBUS,
        HBALL_RS00_INDEX_ROTATION,
    };
    static const uint8_t values[][4] = {
        {0x05U, 0x00U, 0x00U, 0x00U},
        {0x00U, 0x00U, 0xa0U, 0x3fU},
        {0x00U, 0x00U, 0x20U, 0xc0U},
        {0x00U, 0x00U, 0x70U, 0x40U},
        {0x00U, 0x00U, 0x42U, 0x42U},
        {0xfdU, 0xffU, 0x00U, 0x00U},
    };
    hball_motor_monitor_t monitor;
    hball_can_frame_t request;
    hball_can_frame_t reply;
    size_t index;

    hball_motor_monitor_init(&monitor, HBALL_RS00_MOTOR_ID);
    for (index = 0U; index < sizeof(indexes) / sizeof(indexes[0]); ++index)
    {
        assert(hball_motor_monitor_make_parameter_read(
            &monitor, indexes[index], (uint32_t)(100U + index), &request
        ));
        hball_make_parameter_reply(&reply, indexes[index], values[index]);
        assert(hball_motor_monitor_accept(
            &monitor, &reply, (uint32_t)(101U + index)
        ) == HBALL_CAN_EVENT_PARAMETER);
        assert(!monitor.parameter_pending);
    }

    assert(monitor.parameters.valid_flags == HBALL_RS00_PARAMETER_VALID_ALL);
    assert(monitor.parameters.run_mode == 5U);
    assert(fabsf(monitor.parameters.mech_position_rad - 1.25F) < 1.0e-7F);
    assert(fabsf(monitor.parameters.filtered_iq_a - (-2.5F)) < 1.0e-7F);
    assert(fabsf(monitor.parameters.mech_velocity_rad_s - 3.75F) < 1.0e-7F);
    assert(fabsf(monitor.parameters.vbus_v - 48.5F) < 1.0e-7F);
    assert(monitor.parameters.rotation == -3);
    assert(monitor.parameter_rx_total == 6U);
    assert(hball_motor_monitor_parameters_fresh(&monitor, 110U, 20U));
    assert(!hball_motor_monitor_parameters_fresh(&monitor, 130U, 20U));
}

static void test_parameter_reply_validation_and_timeout_fail_closed(void)
{
    const uint8_t value[4] = {0x00U, 0x00U, 0x80U, 0x3fU};
    hball_motor_monitor_t monitor;
    hball_can_frame_t request;
    hball_can_frame_t reply;

    hball_motor_monitor_init(&monitor, HBALL_RS00_MOTOR_ID);
    assert(hball_motor_monitor_make_parameter_read(
        &monitor, HBALL_RS00_INDEX_MECH_POSITION, 10U, &request
    ));
    hball_make_parameter_reply(
        &reply, HBALL_RS00_INDEX_MECH_POSITION, value
    );
    reply.id = hball_rs00_ext_id(
        HBALL_RS00_TYPE_GET_SINGLE_PARAMETER, HBALL_RS00_MOTOR_ID, 0xaaU
    );
    assert(hball_motor_monitor_accept(&monitor, &reply, 11U)
        == HBALL_CAN_EVENT_IGNORED);
    assert(monitor.parameter_pending);

    reply.id = hball_rs00_ext_id(
        HBALL_RS00_TYPE_GET_SINGLE_PARAMETER,
        HBALL_RS00_MOTOR_ID,
        HBALL_RS00_MASTER_ID
    );
    reply.dlc = 7U;
    assert(hball_motor_monitor_accept(&monitor, &reply, 12U)
        == HBALL_CAN_EVENT_INVALID);
    assert(monitor.parameter_pending);

    reply.dlc = 8U;
    reply.data[0] = 0x16U;
    assert(hball_motor_monitor_accept(&monitor, &reply, 13U)
        == HBALL_CAN_EVENT_INVALID);
    assert(monitor.parameter_pending);

    assert(!hball_motor_monitor_expire_parameter_request(
        &monitor, 19U, 10U
    ));
    assert(hball_motor_monitor_expire_parameter_request(
        &monitor, 21U, 10U
    ));
    assert(!monitor.parameter_pending);
    assert(monitor.parameter_timeout_total == 1U);
}

static void test_unknown_error_contract_is_preserved_as_raw_bytes(void)
{
    hball_motor_monitor_t monitor;
    hball_can_frame_t frame;
    const uint8_t raw[8] = {0xdeU, 0xadU, 0xbeU, 0xefU, 1U, 2U, 3U, 4U};

    hball_motor_monitor_init(&monitor, HBALL_RS00_MOTOR_ID);
    memset(&frame, 0, sizeof(frame));
    frame.id = hball_rs00_ext_id(
        HBALL_RS00_TYPE_ERROR_FEEDBACK,
        HBALL_RS00_MOTOR_ID,
        HBALL_RS00_MASTER_ID
    );
    frame.is_extended = 1U;
    frame.dlc = 8U;
    memcpy(frame.data, raw, sizeof(raw));

    assert(hball_motor_monitor_accept(&monitor, &frame, 1U)
        == HBALL_CAN_EVENT_ERROR_RAW);
    assert(monitor.error_raw_total == 1U);
    assert(monitor.last_error_raw_id == frame.id);
    assert(monitor.last_error_raw_dlc == 8U);
    assert(memcmp(monitor.last_error_raw_data, raw, sizeof(raw)) == 0);
}

static void test_mspm0_imu_frames_decode_fixed_point_si_units(void)
{
    hball_msp_monitor_t monitor;
    hball_can_frame_t accel = {
        HBALL_MSP_CAN_ID_ACCEL, 0U, 0U, 8U,
        {0x34U, 0x12U, 0xe8U, 0x03U, 0x06U, 0xffU, 0x4fU, 0x26U}
    };
    hball_can_frame_t gyro = {
        HBALL_MSP_CAN_ID_GYRO, 0U, 0U, 8U,
        {0x34U, 0x12U, 0x9cU, 0xffU, 0xc8U, 0x00U, 0xd4U, 0xfeU}
    };
    hball_can_frame_t attitude = {
        HBALL_MSP_CAN_ID_ATTITUDE, 0U, 0U, 8U,
        {0x78U, 0x56U, 0x0aU, 0x00U, 0xecU, 0xffU, 0x2cU, 0x01U}
    };

    hball_msp_monitor_init(&monitor);
    assert(hball_msp_monitor_accept(&monitor, &accel, 100U)
        == HBALL_MSP_EVENT_ACCEL);
    assert(hball_msp_monitor_accept(&monitor, &gyro, 101U)
        == HBALL_MSP_EVENT_GYRO);
    assert(hball_msp_monitor_accept(&monitor, &attitude, 102U)
        == HBALL_MSP_EVENT_ATTITUDE);
    assert(monitor.accel_sequence == 0x1234U);
    assert(monitor.gyro_sequence == 0x1234U);
    assert(monitor.attitude_sequence == 0x5678U);
    assert(fabsf(monitor.accel_mps2[0] - 1.0F) < 1.0e-6F);
    assert(fabsf(monitor.accel_mps2[1] - (-0.25F)) < 1.0e-6F);
    assert(fabsf(monitor.accel_mps2[2] - 9.807F) < 1.0e-6F);
    assert(fabsf(monitor.gyro_rad_s[0] - (-0.1F)) < 1.0e-6F);
    assert(fabsf(monitor.gyro_rad_s[1] - 0.2F) < 1.0e-6F);
    assert(fabsf(monitor.gyro_rad_s[2] - (-0.3F)) < 1.0e-6F);
    assert(fabsf(monitor.attitude_rad[0] - 0.01F) < 1.0e-6F);
    assert(fabsf(monitor.attitude_rad[1] - (-0.02F)) < 1.0e-6F);
    assert(fabsf(monitor.attitude_rad[2] - 0.3F) < 1.0e-6F);
    assert(hball_msp_monitor_imu_fresh(&monitor, 119U, 20U));
    assert(!hball_msp_monitor_imu_fresh(&monitor, 123U, 20U));
}

static void test_mspm0_heartbeat_and_wheel_frames_decode_without_motion_output(void)
{
    hball_msp_monitor_t monitor;
    hball_can_frame_t heartbeat = {
        HBALL_MSP_CAN_ID_HEARTBEAT, 0U, 0U, 8U,
        {0x02U, 0x00U, 0x05U, 0x00U, 0x78U, 0x56U, 0x34U, 0x12U}
    };
    hball_can_frame_t wheel = {
        HBALL_MSP_CAN_ID_WHEEL, 0U, 0U, 8U,
        {0x09U, 0x00U, 0x64U, 0x00U, 0x9cU, 0xffU, 0x00U, 0x00U}
    };

    hball_msp_monitor_init(&monitor);
    assert(hball_msp_monitor_accept(&monitor, &heartbeat, 50U)
        == HBALL_MSP_EVENT_HEARTBEAT);
    assert(hball_msp_monitor_accept(&monitor, &wheel, 55U)
        == HBALL_MSP_EVENT_WHEEL);
    assert(monitor.heartbeat_sequence == 2U);
    assert(monitor.status_flags == 5U);
    assert(monitor.uptime_ms == UINT32_C(0x12345678));
    assert(monitor.wheel_sequence == 9U);
    assert(fabsf(monitor.wheel_left_mps - 0.1F) < 1.0e-6F);
    assert(fabsf(monitor.wheel_right_mps - (-0.1F)) < 1.0e-6F);
    assert(fabsf(monitor.body_speed_mps) < 1.0e-6F);
    assert(hball_msp_monitor_heartbeat_fresh(&monitor, 149U, 100U));
    assert(!hball_msp_monitor_heartbeat_fresh(&monitor, 151U, 100U));
}

static void test_mspm0_imu_source_timestamp_frame_is_retained(void)
{
    hball_msp_monitor_t monitor;
    hball_can_frame_t frame = {
        HBALL_MSP_CAN_ID_IMU_TIME, 0U, 0U, 8U,
        {0x34U, 0x12U, 0x0fU, 0x00U, 0x78U, 0x56U, 0x34U, 0x12U}
    };

    hball_msp_monitor_init(&monitor);
    assert(hball_msp_monitor_accept(&monitor, &frame, 321U)
        == HBALL_MSP_EVENT_IMU_TIME);
    assert(monitor.imu_time_valid);
    assert(monitor.imu_epoch == UINT16_C(0x1234));
    assert(monitor.imu_sample_mask == UINT16_C(0x000f));
    assert(monitor.imu_source_time_ms == UINT32_C(0x12345678));
    assert(monitor.last_imu_time_ms == 321U);
}

static void test_mspm0_rejects_extended_remote_or_wrong_length_frames(void)
{
    hball_msp_monitor_t monitor;
    hball_can_frame_t frame = {
        HBALL_MSP_CAN_ID_ACCEL, 1U, 0U, 8U, {0U}
    };

    hball_msp_monitor_init(&monitor);
    assert(hball_msp_monitor_accept(&monitor, &frame, 1U)
        == HBALL_MSP_EVENT_IGNORED);
    frame.is_extended = 0U;
    frame.is_remote = 1U;
    assert(hball_msp_monitor_accept(&monitor, &frame, 2U)
        == HBALL_MSP_EVENT_INVALID);
    frame.is_remote = 0U;
    frame.dlc = 7U;
    assert(hball_msp_monitor_accept(&monitor, &frame, 3U)
        == HBALL_MSP_EVENT_INVALID);
    assert(monitor.rx_total == 3U);
    assert(monitor.rx_invalid == 2U);
    assert(monitor.rx_ignored == 1U);
}

static void test_mspm0_sequence_gate_rejects_duplicate_and_out_of_order_data(void)
{
    hball_msp_monitor_t monitor;
    hball_can_frame_t accel = {
        HBALL_MSP_CAN_ID_ACCEL, 0U, 0U, 8U,
        {0x0aU, 0x00U, 0xe8U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U}
    };

    hball_msp_monitor_init(&monitor);
    assert(hball_msp_monitor_accept(&monitor, &accel, 100U)
        == HBALL_MSP_EVENT_ACCEL);
    assert(fabsf(monitor.accel_mps2[0] - 1.0F) < 1.0e-6F);

    accel.data[2] = 0xd0U;
    accel.data[3] = 0x07U;
    assert(hball_msp_monitor_accept(&monitor, &accel, 110U)
        == HBALL_MSP_EVENT_DUPLICATE);
    assert(monitor.last_accel_ms == 100U);
    assert(fabsf(monitor.accel_mps2[0] - 1.0F) < 1.0e-6F);

    accel.data[0] = 0x09U;
    accel.data[2] = 0xb8U;
    accel.data[3] = 0x0bU;
    assert(hball_msp_monitor_accept(&monitor, &accel, 120U)
        == HBALL_MSP_EVENT_OUT_OF_ORDER);
    assert(monitor.accel_sequence == 10U);
    assert(monitor.last_accel_ms == 100U);
    assert(fabsf(monitor.accel_mps2[0] - 1.0F) < 1.0e-6F);
    assert(monitor.rx_duplicate == 1U);
    assert(monitor.rx_out_of_order == 1U);
}

static void test_mspm0_sequence_gate_counts_gaps_and_accepts_wraparound(void)
{
    hball_msp_monitor_t monitor;
    hball_can_frame_t gyro = {
        HBALL_MSP_CAN_ID_GYRO, 0U, 0U, 8U,
        {0xfeU, 0xffU, 0x00U, 0x00U, 0x00U, 0x00U, 0x64U, 0x00U}
    };

    hball_msp_monitor_init(&monitor);
    assert(hball_msp_monitor_accept(&monitor, &gyro, 1U)
        == HBALL_MSP_EVENT_GYRO);
    gyro.data[0] = 0x01U;
    gyro.data[1] = 0x00U;
    assert(hball_msp_monitor_accept(&monitor, &gyro, 2U)
        == HBALL_MSP_EVENT_GYRO);
    assert(monitor.gyro_sequence == 1U);
    assert(monitor.rx_gap == 2U);
    assert(monitor.rx_duplicate == 0U);
    assert(monitor.rx_out_of_order == 0U);
}

static void test_mspm0_reboot_resets_sequence_gate_without_hiding_diagnostics(void)
{
    hball_msp_monitor_t monitor;
    hball_can_frame_t heartbeat = {
        HBALL_MSP_CAN_ID_HEARTBEAT, 0U, 0U, 8U,
        {0x64U, 0x00U, 0x02U, 0x00U, 0xa0U, 0x86U, 0x01U, 0x00U}
    };
    hball_can_frame_t accel = {
        HBALL_MSP_CAN_ID_ACCEL, 0U, 0U, 8U,
        {0xc8U, 0x00U, 0xe8U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U}
    };

    hball_msp_monitor_init(&monitor);
    assert(hball_msp_monitor_accept(&monitor, &heartbeat, 100U)
        == HBALL_MSP_EVENT_HEARTBEAT);
    assert(hball_msp_monitor_accept(&monitor, &accel, 101U)
        == HBALL_MSP_EVENT_ACCEL);

    heartbeat.data[0] = 0x00U;
    heartbeat.data[1] = 0x00U;
    heartbeat.data[4] = 0x05U;
    heartbeat.data[5] = 0x00U;
    heartbeat.data[6] = 0x00U;
    heartbeat.data[7] = 0x00U;
    assert(hball_msp_monitor_accept(&monitor, &heartbeat, 200U)
        == HBALL_MSP_EVENT_HEARTBEAT);
    assert(monitor.reboot_total == 1U);
    assert(monitor.heartbeat_valid);
    assert(!monitor.accel_valid);
    assert(!monitor.gyro_valid);
    assert(!monitor.wheel_valid);
    assert(!monitor.attitude_valid);

    accel.data[0] = 0x00U;
    accel.data[1] = 0x00U;
    assert(hball_msp_monitor_accept(&monitor, &accel, 201U)
        == HBALL_MSP_EVENT_ACCEL);
    assert(monitor.accel_sequence == 0U);
}

int main(void)
{
    test_get_id_frame_and_probe_reply();
    test_feedback_decode_and_freshness();
    test_invalid_or_wrong_host_frames_are_not_feedback();
    test_parameter_read_request_is_strictly_whitelisted();
    test_parameter_replies_decode_all_required_motor_data();
    test_parameter_reply_validation_and_timeout_fail_closed();
    test_unknown_error_contract_is_preserved_as_raw_bytes();
    test_mspm0_imu_frames_decode_fixed_point_si_units();
    test_mspm0_heartbeat_and_wheel_frames_decode_without_motion_output();
    test_mspm0_imu_source_timestamp_frame_is_retained();
    test_mspm0_rejects_extended_remote_or_wrong_length_frames();
    test_mspm0_sequence_gate_rejects_duplicate_and_out_of_order_data();
    test_mspm0_sequence_gate_counts_gaps_and_accepts_wraparound();
    test_mspm0_reboot_resets_sequence_gate_without_hiding_diagnostics();
    return 0;
}

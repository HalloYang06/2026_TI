#include "hball_control_pipeline.h"
#include "hball_sensor_fusion.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xffU);
    data[1] = (uint8_t)(value >> 8U);
}

static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xffU);
    data[1] = (uint8_t)((value >> 8U) & 0xffU);
    data[2] = (uint8_t)((value >> 16U) & 0xffU);
    data[3] = (uint8_t)(value >> 24U);
}

static hball_can_frame_t make_vector_frame(
    uint32_t id,
    uint16_t sequence,
    int16_t x_milli,
    int16_t y_milli,
    int16_t z_milli
)
{
    hball_can_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.dlc = 8U;
    write_u16_le(frame.data, sequence);
    write_u16_le(frame.data + 2U, (uint16_t)x_milli);
    write_u16_le(frame.data + 4U, (uint16_t)y_milli);
    write_u16_le(frame.data + 6U, (uint16_t)z_milli);
    return frame;
}

static hball_can_frame_t make_heartbeat_frame(uint16_t status_flags)
{
    hball_can_frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.id = HBALL_MSP_CAN_ID_HEARTBEAT;
    frame.dlc = 8U;
    write_u16_le(frame.data, 1U);
    write_u16_le(frame.data + 2U, status_flags);
    write_u32_le(frame.data + 4U, 1000U);
    return frame;
}

static hball_can_frame_t make_motor_feedback_frame(void)
{
    hball_can_frame_t frame;
    const uint16_t data2 = (uint16_t)((2U << 14) | HBALL_RS00_MOTOR_ID);

    memset(&frame, 0, sizeof(frame));
    frame.id = hball_rs00_ext_id(
        HBALL_RS00_TYPE_FEEDBACK, data2, HBALL_RS00_MASTER_ID
    );
    frame.is_extended = 1U;
    frame.dlc = 8U;
    frame.data[0] = 0x80U;
    frame.data[2] = 0x80U;
    frame.data[4] = 0x80U;
    frame.data[7] = 0xfaU;
    return frame;
}

static hball_sensor_snapshot_t make_snapshot_from_can(
    int16_t yaw_rate_millirad_s, uint16_t status_flags
)
{
    hball_sensor_fusion_t fusion;
    hball_sensor_snapshot_t snapshot;
    hball_vision_measurement_t vision;
    hball_msp_monitor_t msp;
    hball_motor_monitor_t motor;
    hball_can_frame_t heartbeat = make_heartbeat_frame(status_flags);
    hball_can_frame_t accel = make_vector_frame(
        HBALL_MSP_CAN_ID_ACCEL, 1U, 0, 0, 9807
    );
    hball_can_frame_t gyro = make_vector_frame(
        HBALL_MSP_CAN_ID_GYRO, 1U, 0, 0, yaw_rate_millirad_s
    );
    hball_can_frame_t attitude = make_vector_frame(
        HBALL_MSP_CAN_ID_ATTITUDE, 1U, 0, 0, 0
    );
    hball_can_frame_t wheel = make_vector_frame(
        HBALL_MSP_CAN_ID_WHEEL, 1U, 0, 0, 0
    );
    hball_can_frame_t motor_frame = make_motor_feedback_frame();

    hball_msp_monitor_init(&msp);
    assert(hball_msp_monitor_accept(&msp, &heartbeat, 100U)
        == HBALL_MSP_EVENT_HEARTBEAT);
    assert(hball_msp_monitor_accept(&msp, &accel, 101U)
        == HBALL_MSP_EVENT_ACCEL);
    assert(hball_msp_monitor_accept(&msp, &gyro, 102U)
        == HBALL_MSP_EVENT_GYRO);
    assert(hball_msp_monitor_accept(&msp, &attitude, 103U)
        == HBALL_MSP_EVENT_ATTITUDE);
    assert(hball_msp_monitor_accept(&msp, &wheel, 104U)
        == HBALL_MSP_EVENT_WHEEL);

    hball_motor_monitor_init(&motor, HBALL_RS00_MOTOR_ID);
    assert(hball_motor_monitor_accept(&motor, &motor_frame, 105U)
        == HBALL_CAN_EVENT_FEEDBACK);

    hball_sensor_fusion_init(&fusion);
    hball_sensor_fusion_set_msp(&fusion, &msp);
    hball_sensor_fusion_set_motor(&fusion, &motor.feedback, 105U);
    memset(&vision, 0, sizeof(vision));
    vision.flags = HBALL_VISION_FLAG_DETECTED
        | HBALL_VISION_FLAG_POSITION_VALID;
    vision.sequence = 1U;
    vision.capture_time_us = UINT64_C(100000);
    vision.confidence = 0.95F;
    hball_sensor_fusion_set_vision(&fusion, &vision, 106U);
    hball_sensor_fusion_snapshot(&fusion, 110U, &snapshot);
    return snapshot;
}

static hball_control_output_t run_pipeline(
    const hball_sensor_snapshot_t *snapshot
)
{
    hball_control_pipeline_t pipeline;
    hball_control_output_t output;

    hball_control_pipeline_init(&pipeline, 0.0F);
    hball_control_pipeline_step(&pipeline, snapshot, 0.005F, 0.0F, &output);
    return output;
}

static void test_can_measurements_reach_lqg_shadow_without_motion_output(void)
{
    hball_sensor_snapshot_t straight = make_snapshot_from_can(
        0, HBALL_MSP_STATUS_IMU_VALID | HBALL_MSP_STATUS_CHASSIS_READY
    );
    hball_sensor_snapshot_t turning = make_snapshot_from_can(
        1000, HBALL_MSP_STATUS_IMU_VALID | HBALL_MSP_STATUS_CHASSIS_READY
    );
    const hball_control_output_t straight_output = run_pipeline(&straight);
    const hball_control_output_t turning_output = run_pipeline(&turning);

    assert((turning.valid_flags & HBALL_SENSOR_VALID_IMU) != 0U);
    assert((turning.valid_flags & HBALL_SENSOR_VALID_MOTOR) != 0U);
    assert((turning.valid_flags & HBALL_SENSOR_VALID_HEARTBEAT) != 0U);
    assert(fabsf(turning.yaw_rate_rad_s - 1.0F) < 1.0e-6F);
    assert(turning_output.mode == HBALL_CONTROL_TRACKING);
    assert(turning_output.safety_eligible);
    assert(isfinite(turning_output.shadow_command_rad));
    assert(fabsf(turning_output.shadow_command_rad) <= 0.069814F);
    assert(fabsf(
        turning_output.shadow_command_rad - straight_output.shadow_command_rad
    ) > 1.0e-4F);
}

static void test_can_heartbeat_health_bit_blocks_lqg_safety_eligibility(void)
{
    const hball_sensor_snapshot_t snapshot = make_snapshot_from_can(
        1000, HBALL_MSP_STATUS_CHASSIS_READY
    );
    const hball_control_output_t output = run_pipeline(&snapshot);

    assert(snapshot.imu_age_ms <= HBALL_SENSOR_IMU_STALE_MS);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_HEARTBEAT) != 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_IMU) == 0U);
    assert(!output.safety_eligible);
    assert(isfinite(output.shadow_command_rad));
}

int main(void)
{
    test_can_measurements_reach_lqg_shadow_without_motion_output();
    test_can_heartbeat_health_bit_blocks_lqg_safety_eligibility();
    return 0;
}

#include "hball_sensor_fusion.h"

#include <assert.h>
#include <math.h>
#include <string.h>

static hball_sensor_fusion_t make_populated_fusion(void)
{
    hball_sensor_fusion_t fusion;
    hball_vision_measurement_t vision;
    hball_msp_monitor_t msp;
    hball_motor_feedback_t motor;

    hball_sensor_fusion_init(&fusion);
    memset(&vision, 0, sizeof(vision));
    vision.flags = HBALL_VISION_FLAG_DETECTED
        | HBALL_VISION_FLAG_POSITION_VALID;
    vision.sequence = 12U;
    vision.capture_time_us = UINT64_C(987654321);
    vision.ball_position_m = 0.025F;
    vision.confidence = 0.91F;
    hball_sensor_fusion_set_vision(&fusion, &vision, 100U);

    hball_msp_monitor_init(&msp);
    msp.heartbeat_valid = true;
    msp.accel_valid = true;
    msp.gyro_valid = true;
    msp.attitude_valid = true;
    msp.wheel_valid = true;
    msp.status_flags = HBALL_MSP_STATUS_ESTOP_ACTIVE
        | HBALL_MSP_STATUS_IMU_VALID;
    msp.accel_sequence = 20U;
    msp.gyro_sequence = 20U;
    msp.attitude_sequence = 10U;
    msp.wheel_sequence = 8U;
    msp.accel_mps2[0] = 0.4F;
    msp.accel_mps2[1] = -0.2F;
    msp.gyro_rad_s[2] = 0.6F;
    msp.attitude_rad[1] = -0.03F;
    msp.body_speed_mps = 0.5F;
    msp.last_heartbeat_ms = 80U;
    msp.last_accel_ms = 95U;
    msp.last_gyro_ms = 96U;
    msp.last_attitude_ms = 90U;
    msp.last_wheel_ms = 85U;
    hball_sensor_fusion_set_msp(&fusion, &msp);

    memset(&motor, 0, sizeof(motor));
    motor.position_rad = 0.1F;
    motor.velocity_rad_s = -0.2F;
    motor.torque_nm = 0.3F;
    motor.fault_summary = 4U;
    hball_sensor_fusion_set_motor(&fusion, &motor, 99U);
    return fusion;
}

static void test_snapshot_combines_fresh_usb_can_and_motor_sources(void)
{
    hball_sensor_fusion_t fusion = make_populated_fusion();
    hball_sensor_snapshot_t snapshot;

    hball_sensor_fusion_snapshot(&fusion, 110U, &snapshot);

    assert(snapshot.sequence == 1U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) != 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_IMU) != 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_WHEEL) != 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_MOTOR) != 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_HEARTBEAT) != 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_ESTOP_ACTIVE) != 0U);
    assert(snapshot.vision_sequence == 12U);
    assert(snapshot.vision_capture_time_us == UINT64_C(987654321));
    assert(snapshot.vision_receive_age_ms == 10U);
    assert(snapshot.imu_age_ms == 20U);
    assert(snapshot.motor_age_ms == 11U);
    assert(fabsf(snapshot.ball_position_m - 0.025F) < 1.0e-7F);
    assert(fabsf(snapshot.body_pitch_rad - (-0.03F)) < 1.0e-7F);
    assert(fabsf(snapshot.yaw_rate_rad_s - 0.6F) < 1.0e-7F);
    assert(fabsf(snapshot.motor_angle_rad - 0.1F) < 1.0e-7F);
}

static void test_snapshot_expires_each_source_by_its_own_deadline(void)
{
    hball_sensor_fusion_t fusion = make_populated_fusion();
    hball_sensor_snapshot_t snapshot;

    hball_sensor_fusion_snapshot(&fusion, 131U, &snapshot);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) != 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_HEARTBEAT) != 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_IMU) == 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_WHEEL) == 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_MOTOR) == 0U);

    hball_sensor_fusion_snapshot(&fusion, 201U, &snapshot);
    assert(snapshot.sequence == 2U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) == 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_HEARTBEAT) == 0U);
    assert(snapshot.vision_receive_age_ms == 101U);
}

static void test_invalid_vision_flags_never_enter_valid_snapshot(void)
{
    hball_sensor_fusion_t fusion;
    hball_sensor_snapshot_t snapshot;
    hball_vision_measurement_t vision;

    hball_sensor_fusion_init(&fusion);
    memset(&vision, 0, sizeof(vision));
    vision.sequence = 1U;
    vision.confidence = 0.8F;
    hball_sensor_fusion_set_vision(&fusion, &vision, 5U);
    hball_sensor_fusion_snapshot(&fusion, 5U, &snapshot);

    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_VISION) == 0U);
}

static void test_mspm0_imu_health_bit_gates_fresh_samples(void)
{
    hball_sensor_fusion_t fusion = make_populated_fusion();
    hball_sensor_snapshot_t snapshot;

    fusion.msp.status_flags &= (uint16_t)~HBALL_MSP_STATUS_IMU_VALID;
    hball_sensor_fusion_snapshot(&fusion, 110U, &snapshot);

    assert(snapshot.imu_age_ms == 20U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_HEARTBEAT) != 0U);
    assert((snapshot.valid_flags & HBALL_SENSOR_VALID_IMU) == 0U);
}

int main(void)
{
    test_snapshot_combines_fresh_usb_can_and_motor_sources();
    test_snapshot_expires_each_source_by_its_own_deadline();
    test_invalid_vision_flags_never_enter_valid_snapshot();
    test_mspm0_imu_health_bit_gates_fresh_samples();
    return 0;
}

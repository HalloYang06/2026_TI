#include "hball_sensor_fusion.h"

#include <stddef.h>
#include <string.h>

static uint32_t hball_age_ms(bool valid, uint32_t now_ms, uint32_t then_ms)
{
    return valid ? (uint32_t)(now_ms - then_ms) : UINT32_MAX;
}

static uint32_t hball_max_u32(uint32_t left, uint32_t right)
{
    return left > right ? left : right;
}

void hball_sensor_fusion_init(hball_sensor_fusion_t *fusion)
{
    if (fusion != NULL)
    {
        memset(fusion, 0, sizeof(*fusion));
    }
}

void hball_sensor_fusion_set_vision(
    hball_sensor_fusion_t *fusion,
    const hball_vision_measurement_t *measurement,
    uint32_t receive_ms
)
{
    if ((fusion == NULL) || (measurement == NULL))
    {
        return;
    }
    fusion->vision = *measurement;
    fusion->vision_receive_ms = receive_ms;
    fusion->vision_received = true;
}

void hball_sensor_fusion_set_msp(
    hball_sensor_fusion_t *fusion, const hball_msp_monitor_t *monitor
)
{
    if ((fusion != NULL) && (monitor != NULL))
    {
        fusion->msp = *monitor;
    }
}

void hball_sensor_fusion_set_motor(
    hball_sensor_fusion_t *fusion,
    const hball_motor_feedback_t *feedback,
    uint32_t receive_ms
)
{
    if ((fusion == NULL) || (feedback == NULL))
    {
        return;
    }
    fusion->motor = *feedback;
    fusion->motor_receive_ms = receive_ms;
    fusion->motor_received = true;
}

void hball_sensor_fusion_set_motor_parameters(
    hball_sensor_fusion_t *fusion,
    const hball_motor_parameters_t *parameters
)
{
    if ((fusion != NULL) && (parameters != NULL))
    {
        fusion->motor_parameters = *parameters;
    }
}

static uint32_t hball_imu_age_ms(
    const hball_msp_monitor_t *monitor, uint32_t now_ms
)
{
    uint32_t age;

    if (!monitor->accel_valid || !monitor->gyro_valid
        || !monitor->attitude_valid)
    {
        return UINT32_MAX;
    }
    age = (uint32_t)(now_ms - monitor->last_accel_ms);
    age = hball_max_u32(age, (uint32_t)(now_ms - monitor->last_gyro_ms));
    return hball_max_u32(
        age, (uint32_t)(now_ms - monitor->last_attitude_ms)
    );
}

void hball_sensor_fusion_snapshot(
    hball_sensor_fusion_t *fusion,
    uint32_t now_ms,
    hball_sensor_snapshot_t *snapshot
)
{
    if ((fusion == NULL) || (snapshot == NULL))
    {
        return;
    }
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->sequence = ++fusion->snapshot_sequence;
    snapshot->created_time_ms = now_ms;
    snapshot->vision_sequence = fusion->vision.sequence;
    snapshot->vision_capture_time_us = fusion->vision.capture_time_us;
    snapshot->vision_receive_age_ms = hball_age_ms(
        fusion->vision_received, now_ms, fusion->vision_receive_ms
    );
    snapshot->imu_age_ms = hball_imu_age_ms(&fusion->msp, now_ms);
    snapshot->wheel_age_ms = hball_age_ms(
        fusion->msp.wheel_valid, now_ms, fusion->msp.last_wheel_ms
    );
    snapshot->motor_age_ms = hball_age_ms(
        fusion->motor_received, now_ms, fusion->motor_receive_ms
    );
    snapshot->heartbeat_age_ms = hball_age_ms(
        fusion->msp.heartbeat_valid, now_ms, fusion->msp.last_heartbeat_ms
    );

    if (fusion->vision_received
        && ((fusion->vision.flags & HBALL_VISION_FLAG_POSITION_VALID) != 0U)
        && (snapshot->vision_receive_age_ms <= HBALL_SENSOR_VISION_STALE_MS))
    {
        snapshot->valid_flags |= HBALL_SENSOR_VALID_VISION;
    }
    if ((snapshot->imu_age_ms <= HBALL_SENSOR_IMU_STALE_MS)
        && ((fusion->msp.status_flags & HBALL_MSP_STATUS_IMU_VALID) != 0U))
    {
        snapshot->valid_flags |= HBALL_SENSOR_VALID_IMU;
    }
    if (snapshot->wheel_age_ms <= HBALL_SENSOR_WHEEL_STALE_MS)
    {
        snapshot->valid_flags |= HBALL_SENSOR_VALID_WHEEL;
    }
    if (snapshot->motor_age_ms <= HBALL_SENSOR_MOTOR_STALE_MS)
    {
        snapshot->valid_flags |= HBALL_SENSOR_VALID_MOTOR;
    }
    if (hball_motor_parameters_fresh(
            &fusion->motor_parameters,
            now_ms,
            HBALL_SENSOR_MOTOR_PARAMETER_STALE_MS))
    {
        snapshot->valid_flags |= HBALL_SENSOR_VALID_MOTOR_PARAMETERS;
    }
    if (snapshot->heartbeat_age_ms <= HBALL_SENSOR_HEARTBEAT_STALE_MS)
    {
        snapshot->valid_flags |= HBALL_SENSOR_VALID_HEARTBEAT;
    }
    if ((fusion->msp.status_flags & HBALL_MSP_STATUS_ESTOP_ACTIVE) != 0U)
    {
        snapshot->valid_flags |= HBALL_SENSOR_ESTOP_ACTIVE;
    }

    snapshot->accel_sequence = fusion->msp.accel_sequence;
    snapshot->gyro_sequence = fusion->msp.gyro_sequence;
    snapshot->attitude_sequence = fusion->msp.attitude_sequence;
    snapshot->wheel_sequence = fusion->msp.wheel_sequence;
    snapshot->msp_status_flags = fusion->msp.status_flags;
    snapshot->motor_fault_summary = fusion->motor.fault_summary;
    snapshot->motor_mode_state = fusion->motor.mode_state;
    snapshot->motor_run_mode = fusion->motor_parameters.run_mode;
    snapshot->ball_position_m = fusion->vision.ball_position_m;
    snapshot->vision_confidence = fusion->vision.confidence;
    snapshot->longitudinal_accel_mps2 = fusion->msp.accel_mps2[0];
    snapshot->lateral_accel_mps2 = fusion->msp.accel_mps2[1];
    snapshot->body_pitch_rad = fusion->msp.attitude_rad[1];
    snapshot->yaw_rate_rad_s = fusion->msp.gyro_rad_s[2];
    snapshot->body_speed_mps = fusion->msp.body_speed_mps;
    if ((snapshot->valid_flags & HBALL_SENSOR_VALID_MOTOR) != 0U)
    {
        snapshot->motor_angle_rad = fusion->motor.position_rad;
        snapshot->motor_velocity_rad_s = fusion->motor.velocity_rad_s;
    }
    else if ((snapshot->valid_flags & HBALL_SENSOR_VALID_MOTOR_PARAMETERS) != 0U)
    {
        snapshot->motor_angle_rad = fusion->motor_parameters.mech_position_rad;
        snapshot->motor_velocity_rad_s =
            fusion->motor_parameters.mech_velocity_rad_s;
    }
    snapshot->motor_torque_nm = fusion->motor.torque_nm;
    snapshot->motor_temperature_c = fusion->motor.temperature_c;
    snapshot->motor_filtered_iq_a = fusion->motor_parameters.filtered_iq_a;
    snapshot->motor_vbus_v = fusion->motor_parameters.vbus_v;
}

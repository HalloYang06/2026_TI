#include "hball_control_pipeline.h"

#include <assert.h>
#include <math.h>
#include <string.h>

static hball_sensor_snapshot_t make_tracking_snapshot(void)
{
    hball_sensor_snapshot_t snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.valid_flags = HBALL_SENSOR_VALID_VISION
        | HBALL_SENSOR_VALID_IMU
        | HBALL_SENSOR_VALID_MOTOR
        | HBALL_SENSOR_VALID_HEARTBEAT;
    snapshot.vision_receive_age_ms = 8U;
    snapshot.vision_confidence = 0.9F;
    snapshot.ball_position_m = 0.004F;
    snapshot.longitudinal_accel_mps2 = 0.1F;
    snapshot.lateral_accel_mps2 = -0.05F;
    snapshot.body_pitch_rad = 0.002F;
    snapshot.yaw_rate_rad_s = 0.3F;
    snapshot.motor_angle_rad = 0.01F;
    return snapshot;
}

static void test_200_hz_pipeline_fuses_each_120_hz_vision_sequence_once(void)
{
    static const uint32_t vision_sequences[10] = {
        1U, 1U, 2U, 2U, 3U, 4U, 4U, 5U, 5U, 6U
    };
    hball_control_pipeline_t pipeline;
    hball_control_output_t output;
    hball_sensor_snapshot_t snapshot = make_tracking_snapshot();

    hball_control_pipeline_init(&pipeline, 0.004F);
    assert(hball_control_pipeline_set_motor_level(&pipeline, 0.0F));
    for (uint32_t step = 0U; step < 10U; ++step)
    {
        snapshot.sequence = step + 1U;
        snapshot.vision_sequence = vision_sequences[step];
        snapshot.ball_position_m = 0.004F + 0.00002F * (float)step;
        hball_control_pipeline_step(
            &pipeline, &snapshot, 0.005F, 0.0F, &output
        );
    }

    assert(pipeline.step_total == 10U);
    assert(pipeline.vision_measurements_consumed == 6U);
    assert(pipeline.duplicate_vision_skips == 4U);
    assert(pipeline.controller.accepted_camera_updates == 6U);
    assert(output.mode == HBALL_CONTROL_TRACKING);
    assert(output.safety_eligible);
    assert(output.linkage_valid);
    assert(output.linkage_calibrated);
    assert(isfinite(output.shadow_command_rad));
    assert(isfinite(output.motor_target_rad));
    assert(fabsf(output.shadow_command_rad) <= 0.104720F);
}

static void test_pipeline_degrades_by_vision_age_without_refusing_model_prediction(void)
{
    hball_control_pipeline_t pipeline;
    hball_control_output_t output;
    hball_sensor_snapshot_t snapshot = make_tracking_snapshot();

    hball_control_pipeline_init(&pipeline, 0.0F);
    assert(hball_control_pipeline_set_motor_level(&pipeline, 0.0F));
    snapshot.vision_sequence = 1U;
    hball_control_pipeline_step(&pipeline, &snapshot, 0.005F, 0.0F, &output);
    assert(output.mode == HBALL_CONTROL_TRACKING);

    snapshot.vision_receive_age_ms = 40U;
    hball_control_pipeline_step(&pipeline, &snapshot, 0.005F, 0.0F, &output);
    assert(output.mode == HBALL_CONTROL_COASTING);
    assert(!output.safety_eligible);

    snapshot.vision_receive_age_ms = 75U;
    hball_control_pipeline_step(&pipeline, &snapshot, 0.005F, 0.0F, &output);
    assert(output.mode == HBALL_CONTROL_DEGRADED);

    snapshot.valid_flags &= ~HBALL_SENSOR_VALID_VISION;
    snapshot.vision_receive_age_ms = 101U;
    hball_control_pipeline_step(&pipeline, &snapshot, 0.005F, 0.0F, &output);
    assert(output.mode == HBALL_CONTROL_VISION_LOST);
    assert(pipeline.step_total == 4U);
    assert(isfinite(output.estimated_position_m));
}

static void test_estop_motor_fault_or_low_confidence_blocks_safety_eligibility(void)
{
    hball_control_pipeline_t pipeline;
    hball_control_output_t output;
    hball_sensor_snapshot_t snapshot = make_tracking_snapshot();

    hball_control_pipeline_init(&pipeline, 0.0F);
    assert(hball_control_pipeline_set_motor_level(&pipeline, 0.0F));
    snapshot.vision_sequence = 1U;
    snapshot.valid_flags |= HBALL_SENSOR_ESTOP_ACTIVE;
    hball_control_pipeline_step(&pipeline, &snapshot, 0.005F, 0.0F, &output);
    assert(!output.safety_eligible);

    snapshot.valid_flags &= ~HBALL_SENSOR_ESTOP_ACTIVE;
    snapshot.motor_fault_summary = 1U;
    snapshot.vision_sequence++;
    hball_control_pipeline_step(&pipeline, &snapshot, 0.005F, 0.0F, &output);
    assert(!output.safety_eligible);

    snapshot.motor_fault_summary = 0U;
    snapshot.vision_confidence = 0.2F;
    snapshot.vision_sequence++;
    hball_control_pipeline_step(&pipeline, &snapshot, 0.005F, 0.0F, &output);
    assert(!output.safety_eligible);
    assert(pipeline.low_confidence_vision_skips == 1U);
}

static void test_read_only_motor_parameters_remain_shadow_only(void)
{
    hball_control_pipeline_t pipeline;
    hball_control_output_t output;
    hball_sensor_snapshot_t snapshot = make_tracking_snapshot();

    hball_control_pipeline_init(&pipeline, 0.0F);
    assert(hball_control_pipeline_set_motor_level(&pipeline, 0.0F));
    snapshot.vision_sequence = 1U;
    snapshot.valid_flags &= ~HBALL_SENSOR_VALID_MOTOR;
    snapshot.valid_flags |= HBALL_SENSOR_VALID_MOTOR_PARAMETERS;
    hball_control_pipeline_step(&pipeline, &snapshot, 0.005F, 0.0F, &output);

    assert(output.mode == HBALL_CONTROL_TRACKING);
    assert(isfinite(output.shadow_command_rad));
    assert(!output.safety_eligible);
}

int main(void)
{
    test_200_hz_pipeline_fuses_each_120_hz_vision_sequence_once();
    test_pipeline_degrades_by_vision_age_without_refusing_model_prediction();
    test_estop_motor_fault_or_low_confidence_blocks_safety_eligibility();
    test_read_only_motor_parameters_remain_shadow_only();
    return 0;
}

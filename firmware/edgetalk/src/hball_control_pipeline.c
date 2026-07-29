#include "hball_control_pipeline.h"

#include <stddef.h>
#include <string.h>

void hball_control_pipeline_init(
    hball_control_pipeline_t *pipeline, float initial_position_m
)
{
    if (pipeline == NULL)
    {
        return;
    }
    memset(pipeline, 0, sizeof(*pipeline));
    hball_lqg_init(&pipeline->lqg, initial_position_m);
}

static hball_lqg_input_t hball_pipeline_make_input(
    const hball_sensor_snapshot_t *snapshot
)
{
    hball_lqg_input_t input;

    memset(&input, 0, sizeof(input));
    if ((snapshot->valid_flags & HBALL_SENSOR_VALID_MOTOR) != 0U)
    {
        input.beam_angle_rad = snapshot->motor_angle_rad;
    }
    if ((snapshot->valid_flags & HBALL_SENSOR_VALID_IMU) != 0U)
    {
        input.body_pitch_rad = snapshot->body_pitch_rad;
        input.longitudinal_accel_mps2 = snapshot->longitudinal_accel_mps2;
        input.lateral_accel_mps2 = snapshot->lateral_accel_mps2;
        input.yaw_rate_rad_s = snapshot->yaw_rate_rad_s;
    }
    return input;
}

static hball_control_mode_t hball_pipeline_mode(
    const hball_control_pipeline_t *pipeline,
    const hball_sensor_snapshot_t *snapshot
)
{
    if (!pipeline->vision_sequence_initialized)
    {
        return HBALL_CONTROL_WAITING_FOR_VISION;
    }
    if ((snapshot->valid_flags & HBALL_SENSOR_VALID_VISION) == 0U)
    {
        return HBALL_CONTROL_VISION_LOST;
    }
    if (snapshot->vision_receive_age_ms <= HBALL_CONTROL_TRACKING_MAX_AGE_MS)
    {
        return HBALL_CONTROL_TRACKING;
    }
    if (snapshot->vision_receive_age_ms <= HBALL_CONTROL_COASTING_MAX_AGE_MS)
    {
        return HBALL_CONTROL_COASTING;
    }
    if (snapshot->vision_receive_age_ms <= HBALL_CONTROL_DEGRADED_MAX_AGE_MS)
    {
        return HBALL_CONTROL_DEGRADED;
    }
    return HBALL_CONTROL_VISION_LOST;
}

static void hball_pipeline_consume_vision(
    hball_control_pipeline_t *pipeline,
    const hball_sensor_snapshot_t *snapshot
)
{
    bool accepted;

    if ((snapshot->valid_flags & HBALL_SENSOR_VALID_VISION) == 0U)
    {
        return;
    }
    if (pipeline->vision_sequence_initialized
        && (snapshot->vision_sequence == pipeline->last_vision_sequence))
    {
        pipeline->duplicate_vision_skips++;
        return;
    }
    pipeline->vision_sequence_initialized = true;
    pipeline->last_vision_sequence = snapshot->vision_sequence;
    if (snapshot->vision_confidence < HBALL_CONTROL_MIN_VISION_CONFIDENCE)
    {
        pipeline->low_confidence_vision_skips++;
        return;
    }

    pipeline->vision_measurements_consumed++;
    accepted = hball_lqg_update_delayed_position(
        &pipeline->lqg,
        snapshot->ball_position_m,
        (float)snapshot->vision_receive_age_ms * 0.001F
    );
    if (!accepted)
    {
        pipeline->rejected_vision_measurements++;
    }
}

static bool hball_pipeline_safety_eligible(
    hball_control_mode_t mode,
    const hball_sensor_snapshot_t *snapshot
)
{
    const uint32_t required = HBALL_SENSOR_VALID_VISION
        | HBALL_SENSOR_VALID_IMU
        | HBALL_SENSOR_VALID_MOTOR
        | HBALL_SENSOR_VALID_HEARTBEAT;

    return (mode == HBALL_CONTROL_TRACKING)
        && ((snapshot->valid_flags & required) == required)
        && ((snapshot->valid_flags & HBALL_SENSOR_ESTOP_ACTIVE) == 0U)
        && (snapshot->motor_fault_summary == 0U)
        && (snapshot->vision_confidence >= HBALL_CONTROL_MIN_VISION_CONFIDENCE);
}

void hball_control_pipeline_step(
    hball_control_pipeline_t *pipeline,
    const hball_sensor_snapshot_t *snapshot,
    float dt_s,
    float target_position_m,
    hball_control_output_t *output
)
{
    hball_lqg_input_t input;

    if ((pipeline == NULL) || (snapshot == NULL) || (output == NULL)
        || (dt_s <= 0.0F))
    {
        return;
    }
    input = hball_pipeline_make_input(snapshot);
    hball_lqg_predict(&pipeline->lqg, dt_s, &input);
    hball_pipeline_consume_vision(pipeline, snapshot);

    memset(output, 0, sizeof(*output));
    output->mode = hball_pipeline_mode(pipeline, snapshot);
    output->shadow_command_rad = hball_lqg_command(
        &pipeline->lqg, &input, target_position_m
    );
    output->estimated_position_m = pipeline->lqg.state[0];
    output->estimated_velocity_mps = pipeline->lqg.state[1];
    output->estimated_disturbance_mps2 = pipeline->lqg.state[2];
    output->safety_eligible = hball_pipeline_safety_eligible(
        output->mode, snapshot
    );
    pipeline->step_total++;
}

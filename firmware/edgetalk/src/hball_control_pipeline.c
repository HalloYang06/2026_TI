#include "hball_control_pipeline.h"
#include "hball_deployment_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define HBALL_CONTROL_RELOCK_REJECT_COUNT 3U
#define HBALL_CONTROL_RELOCK_ERROR_M 0.025F

void hball_control_pipeline_init(
    hball_control_pipeline_t *pipeline, float initial_position_m
)
{
    if (pipeline == NULL)
    {
        return;
    }
    memset(pipeline, 0, sizeof(*pipeline));
    hball_deployment_controller_init(
        &pipeline->controller, initial_position_m
    );
    hball_fourbar_default_geometry(&pipeline->fourbar);
}

bool hball_control_pipeline_set_motor_level(
    hball_control_pipeline_t *pipeline, float motor_level_angle_rad
)
{
    if ((pipeline == NULL) || !isfinite(motor_level_angle_rad))
    {
        return false;
    }
    pipeline->motor_level_angle_rad = motor_level_angle_rad;
    pipeline->linkage_calibrated = true;
    return true;
}

static hball_deployment_input_t hball_pipeline_make_input(
    const hball_control_pipeline_t *pipeline,
    const hball_sensor_snapshot_t *snapshot,
    float *actual_pipe_angle_rad,
    bool *linkage_valid
)
{
    hball_deployment_input_t input;

    memset(&input, 0, sizeof(input));
    *linkage_valid = false;
    *actual_pipe_angle_rad =
        pipeline->controller.previous_pipe_command_rad;
    if (pipeline->linkage_calibrated
        && ((snapshot->valid_flags & HBALL_SENSOR_VALID_MOTOR) != 0U))
    {
        const float geometric_motor_angle_rad =
            pipeline->fourbar.geometric_level_motor_angle_rad
            + HBALL_LINKAGE_MOTOR_DIRECTION_SIGN
                * (snapshot->motor_angle_rad
                    - pipeline->motor_level_angle_rad);

        *linkage_valid = hball_fourbar_forward(
            &pipeline->fourbar,
            geometric_motor_angle_rad,
            actual_pipe_angle_rad
        );
    }
    input.pipe_angle_rad = *actual_pipe_angle_rad;
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
    accepted = hball_deployment_controller_update_delayed_position(
        &pipeline->controller,
        snapshot->ball_position_m,
        (float)snapshot->vision_receive_age_ms * 0.001F
    );
    if (!accepted)
    {
        pipeline->rejected_vision_measurements++;
        if (pipeline->consecutive_vision_rejects < UINT8_MAX)
        {
            pipeline->consecutive_vision_rejects++;
        }
        if ((pipeline->consecutive_vision_rejects
             >= HBALL_CONTROL_RELOCK_REJECT_COUNT)
            && (fabsf(snapshot->ball_position_m
                    - pipeline->controller.state[0])
                >= HBALL_CONTROL_RELOCK_ERROR_M))
        {
            hball_deployment_controller_relock_position(
                &pipeline->controller, snapshot->ball_position_m
            );
            pipeline->consecutive_vision_rejects = 0U;
            pipeline->vision_relocks++;
        }
    }
    else
    {
        pipeline->consecutive_vision_rejects = 0U;
    }
}

static bool hball_pipeline_safety_eligible(
    hball_control_mode_t mode,
    const hball_sensor_snapshot_t *snapshot,
    bool linkage_valid,
    bool linkage_calibrated
)
{
    const uint32_t required = HBALL_SENSOR_VALID_VISION
        | HBALL_SENSOR_VALID_IMU
        | HBALL_SENSOR_VALID_MOTOR
        | HBALL_SENSOR_VALID_HEARTBEAT;

    return (mode == HBALL_CONTROL_TRACKING)
        && linkage_valid
        && linkage_calibrated
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
    hball_deployment_input_t input;
    float actual_pipe_angle_rad;
    float motor_offset_rad = 0.0F;
    bool linkage_valid;
    bool inverse_valid;

    if ((pipeline == NULL) || (snapshot == NULL) || (output == NULL)
        || (dt_s <= 0.0F))
    {
        return;
    }
    input = hball_pipeline_make_input(
        pipeline, snapshot, &actual_pipe_angle_rad, &linkage_valid
    );
    hball_deployment_controller_predict(
        &pipeline->controller, dt_s, &input
    );
    hball_pipeline_consume_vision(pipeline, snapshot);

    memset(output, 0, sizeof(*output));
    output->mode = hball_pipeline_mode(pipeline, snapshot);
    output->shadow_command_rad = hball_deployment_controller_command(
        &pipeline->controller,
        &input,
        target_position_m,
        dt_s,
        output->mode == HBALL_CONTROL_TRACKING
    );
    inverse_valid = hball_fourbar_motor_offset(
        &pipeline->fourbar,
        output->shadow_command_rad,
        &motor_offset_rad
    );
    output->estimated_position_m = pipeline->controller.state[0];
    output->estimated_velocity_mps = pipeline->controller.state[1];
    output->estimated_disturbance_mps2 = pipeline->controller.state[2];
    output->actual_pipe_angle_rad = actual_pipe_angle_rad;
    output->linkage_valid = linkage_valid && inverse_valid;
    output->linkage_calibrated = pipeline->linkage_calibrated;
    output->motor_target_rad = pipeline->linkage_calibrated
        ? pipeline->motor_level_angle_rad
            + HBALL_LINKAGE_MOTOR_DIRECTION_SIGN * motor_offset_rad
        : motor_offset_rad;
    output->safety_eligible = hball_pipeline_safety_eligible(
        output->mode,
        snapshot,
        output->linkage_valid,
        output->linkage_calibrated
    );
    pipeline->step_total++;
}

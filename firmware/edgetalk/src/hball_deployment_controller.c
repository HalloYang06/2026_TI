#include "hball_deployment_controller.h"
#include "hball_deployment_config.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define HBALL_GRAVITY_MPS2 9.80665F
#define HBALL_ROLLING_FACTOR (5.0F / 7.0F)
#define HBALL_BEAM_MISALIGNMENT_RAD 0.0F
#define HBALL_VISCOUS_DAMPING 0.20F
#define HBALL_CAMERA_NOISE_STD_M 0.0015F
#define HBALL_INNOVATION_GATE_SIGMA 6.0F
#define HBALL_LQI_POSITION_GAIN 2.64956F
#define HBALL_LQI_VELOCITY_GAIN 1.230000F
#define HBALL_LQI_INTEGRAL_GAIN 0.787185F
#define HBALL_EDGE_POSITION_GAIN 0.80F
#define HBALL_EDGE_VELOCITY_GAIN 0.60F
#define HBALL_INTEGRAL_LIMIT_M_S 0.25F
#define HBALL_CENTER_LIMIT_M \
    (HBALL_DEPLOYMENT_PHYSICAL_HALF_LENGTH_M \
        - HBALL_DEPLOYMENT_BALL_RADIUS_M)
#define HBALL_EDGE_MARGIN_M 0.045F
#define HBALL_NORMAL_ANGLE_RAD 0.069813170F
#define HBALL_RECOVERY_ANGLE_RAD 0.052359878F
#define HBALL_HARD_ANGLE_RAD 0.104719755F
#define HBALL_PIPE_RATE_LIMIT_RAD_S 0.50F
#define HBALL_EDGE_PIPE_RATE_LIMIT_RAD_S 0.75F
#define HBALL_MAX_VALID_DT_S 0.020F
#define HBALL_MAX_CAMERA_DELAY_S 0.150F

static float g_hball_lqi_position_gain = HBALL_LQI_POSITION_GAIN;
static float g_hball_lqi_velocity_gain = HBALL_LQI_VELOCITY_GAIN;
static float g_hball_lqi_integral_gain = HBALL_LQI_INTEGRAL_GAIN;

static float hball_clampf(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

bool hball_deployment_controller_set_gains(
    float position_gain,
    float velocity_gain,
    float integral_gain
)
{
    if (!isfinite(position_gain) || !isfinite(velocity_gain)
        || !isfinite(integral_gain)
        || (position_gain < 0.0F) || (position_gain > 10.0F)
        || (velocity_gain < 0.0F) || (velocity_gain > 10.0F)
        || (integral_gain < 0.0F) || (integral_gain > 10.0F))
    {
        return false;
    }
    g_hball_lqi_position_gain = position_gain;
    g_hball_lqi_velocity_gain = velocity_gain;
    g_hball_lqi_integral_gain = integral_gain;
    return true;
}

static void hball_copy_state(
    float destination_state[3],
    float destination_covariance[3][3],
    float source_state[3],
    float source_covariance[3][3]
)
{
    memcpy(destination_state, source_state, 3U * sizeof(float));
    memcpy(
        destination_covariance,
        source_covariance,
        9U * sizeof(float)
    );
}

static void hball_covariance_predict(
    float covariance[3][3], float transition[3][3]
)
{
    static const float process_noise[3] = {2.0e-11F, 2.0e-7F, 2.0e-6F};
    float temporary[3][3] = {{0.0F}};
    float predicted[3][3] = {{0.0F}};
    uint8_t row;
    uint8_t column;
    uint8_t index;

    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            for (index = 0U; index < 3U; ++index)
            {
                temporary[row][column] +=
                    transition[row][index] * covariance[index][column];
            }
        }
    }
    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            for (index = 0U; index < 3U; ++index)
            {
                predicted[row][column] +=
                    temporary[row][index] * transition[column][index];
            }
        }
        predicted[row][row] += process_noise[row];
    }
    memcpy(covariance, predicted, sizeof(predicted));
}

static void hball_predict_core(
    float state[3],
    float covariance[3][3],
    float dt_s,
    const hball_deployment_input_t *input
)
{
    float transition[3][3] = {{0.0F}};
    const float world_pipe_angle =
        input->pipe_angle_rad + input->body_pitch_rad;
    const float lever_arm_m =
        HBALL_DEPLOYMENT_HINGE_TO_VISION_ZERO_M + state[0];
    const float position_gradient = HBALL_ROLLING_FACTOR
        * input->yaw_rate_rad_s * input->yaw_rate_rad_s;
    const float velocity_gradient = -HBALL_VISCOUS_DAMPING;
    const float acceleration = HBALL_ROLLING_FACTOR
        * (
            HBALL_GRAVITY_MPS2 * sinf(world_pipe_angle)
            - input->longitudinal_accel_mps2 * cosf(world_pipe_angle)
            + input->yaw_rate_rad_s * input->yaw_rate_rad_s * lever_arm_m
            - input->lateral_accel_mps2 * sinf(HBALL_BEAM_MISALIGNMENT_RAD)
        )
        - HBALL_VISCOUS_DAMPING * state[1]
        + state[2];
    const float position = state[0];
    const float velocity = state[1];

    state[0] = position + velocity * dt_s
        + 0.5F * acceleration * dt_s * dt_s;
    state[1] = velocity + acceleration * dt_s;

    transition[0][0] = 1.0F
        + 0.5F * position_gradient * dt_s * dt_s;
    transition[0][1] = dt_s
        + 0.5F * velocity_gradient * dt_s * dt_s;
    transition[0][2] = 0.5F * dt_s * dt_s;
    transition[1][0] = position_gradient * dt_s;
    transition[1][1] = 1.0F + velocity_gradient * dt_s;
    transition[1][2] = dt_s;
    transition[2][2] = 1.0F;
    hball_covariance_predict(covariance, transition);
}

static bool hball_scalar_position_update(
    float state[3],
    float covariance[3][3],
    float measured_position_m
)
{
    const float measurement_variance =
        HBALL_CAMERA_NOISE_STD_M * HBALL_CAMERA_NOISE_STD_M;
    const float innovation = measured_position_m - state[0];
    float innovation_variance = covariance[0][0] + measurement_variance;
    float gain[3];
    float original_row[3];
    uint8_t row;
    uint8_t column;

    if (innovation_variance < 1.0e-12F)
    {
        innovation_variance = 1.0e-12F;
    }
    if ((innovation * innovation / innovation_variance)
        > (HBALL_INNOVATION_GATE_SIGMA * HBALL_INNOVATION_GATE_SIGMA))
    {
        return false;
    }
    memcpy(original_row, covariance[0], sizeof(original_row));
    for (row = 0U; row < 3U; ++row)
    {
        gain[row] = covariance[row][0] / innovation_variance;
        state[row] += gain[row] * innovation;
    }
    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            covariance[row][column] -= gain[row] * original_row[column];
        }
    }
    for (row = 0U; row < 3U; ++row)
    {
        for (column = (uint8_t)(row + 1U); column < 3U; ++column)
        {
            const float symmetric = 0.5F
                * (covariance[row][column] + covariance[column][row]);
            covariance[row][column] = symmetric;
            covariance[column][row] = symmetric;
        }
        if (covariance[row][row] < 1.0e-12F)
        {
            covariance[row][row] = 1.0e-12F;
        }
    }
    return true;
}

void hball_deployment_controller_init(
    hball_deployment_controller_t *controller,
    float initial_position_m
)
{
    if (controller == NULL)
    {
        return;
    }
    memset(controller, 0, sizeof(*controller));
    controller->state[0] = initial_position_m;
    controller->covariance[0][0] = 9.0e-4F;
    controller->covariance[1][1] = 9.0e-2F;
    controller->covariance[2][2] = 4.0e-2F;
}

void hball_deployment_controller_relock_position(
    hball_deployment_controller_t *controller,
    float measured_position_m
)
{
    float previous_pipe_command_rad;
    uint32_t accepted_camera_updates;
    uint32_t rejected_camera_updates;
    uint32_t too_old_camera_updates;
    uint32_t edge_recovery_total;

    if ((controller == NULL) || !isfinite(measured_position_m))
    {
        return;
    }
    previous_pipe_command_rad = controller->previous_pipe_command_rad;
    accepted_camera_updates = controller->accepted_camera_updates;
    rejected_camera_updates = controller->rejected_camera_updates;
    too_old_camera_updates = controller->too_old_camera_updates;
    edge_recovery_total = controller->edge_recovery_total;
    hball_deployment_controller_init(controller, measured_position_m);
    controller->previous_pipe_command_rad = previous_pipe_command_rad;
    controller->accepted_camera_updates = accepted_camera_updates;
    controller->rejected_camera_updates = rejected_camera_updates;
    controller->too_old_camera_updates = too_old_camera_updates;
    controller->edge_recovery_total = edge_recovery_total;
}

void hball_deployment_controller_predict(
    hball_deployment_controller_t *controller,
    float dt_s,
    const hball_deployment_input_t *input
)
{
    hball_deployment_history_t *entry;

    if ((controller == NULL) || (input == NULL)
        || !isfinite(dt_s) || (dt_s <= 0.0F)
        || (dt_s > HBALL_MAX_VALID_DT_S))
    {
        return;
    }
    hball_predict_core(controller->state, controller->covariance, dt_s, input);
    controller->current_time_s += dt_s;

    if (controller->history_count == HBALL_DEPLOYMENT_HISTORY_STEPS)
    {
        memmove(
            &controller->history[0],
            &controller->history[1],
            (HBALL_DEPLOYMENT_HISTORY_STEPS - 1U)
                * sizeof(controller->history[0])
        );
        controller->history_count--;
    }
    entry = &controller->history[controller->history_count++];
    hball_copy_state(
        entry->state,
        entry->covariance,
        controller->state,
        controller->covariance
    );
    entry->input = *input;
    entry->dt_s = dt_s;
    entry->time_s = controller->current_time_s;
}

bool hball_deployment_controller_update_delayed_position(
    hball_deployment_controller_t *controller,
    float measured_position_m,
    float age_s
)
{
    float measurement_time_s;
    float best_delta_s;
    uint8_t best_index;
    uint8_t index;

    if ((controller == NULL) || !isfinite(measured_position_m)
        || !isfinite(age_s) || (age_s < 0.0F)
        || (controller->history_count == 0U))
    {
        return false;
    }
    if (age_s > HBALL_MAX_CAMERA_DELAY_S)
    {
        controller->too_old_camera_updates++;
        return false;
    }
    measurement_time_s = controller->current_time_s - age_s;
    if ((controller->history_count == HBALL_DEPLOYMENT_HISTORY_STEPS)
        && (measurement_time_s
            < controller->history[0].time_s
                - controller->history[0].dt_s))
    {
        controller->too_old_camera_updates++;
        return false;
    }
    best_index = 0U;
    best_delta_s = fabsf(
        controller->history[0].time_s - measurement_time_s
    );
    for (index = 1U; index < controller->history_count; ++index)
    {
        const float delta_s = fabsf(
            controller->history[index].time_s - measurement_time_s
        );
        if (delta_s < best_delta_s)
        {
            best_delta_s = delta_s;
            best_index = index;
        }
    }
    if (!hball_scalar_position_update(
            controller->history[best_index].state,
            controller->history[best_index].covariance,
            measured_position_m))
    {
        controller->rejected_camera_updates++;
        return false;
    }
    for (index = (uint8_t)(best_index + 1U);
         index < controller->history_count;
         ++index)
    {
        hball_copy_state(
            controller->history[index].state,
            controller->history[index].covariance,
            controller->history[index - 1U].state,
            controller->history[index - 1U].covariance
        );
        hball_predict_core(
            controller->history[index].state,
            controller->history[index].covariance,
            controller->history[index].dt_s,
            &controller->history[index].input
        );
    }
    hball_copy_state(
        controller->state,
        controller->covariance,
        controller->history[controller->history_count - 1U].state,
        controller->history[controller->history_count - 1U].covariance
    );
    controller->accepted_camera_updates++;
    return true;
}

static float hball_feedforward(
    const hball_deployment_controller_t *controller,
    const hball_deployment_input_t *input
)
{
    const float centripetal = input->yaw_rate_rad_s
        * input->yaw_rate_rad_s
        * (
            HBALL_DEPLOYMENT_HINGE_TO_VISION_ZERO_M
            + controller->state[0]
        );
    const float lateral = input->lateral_accel_mps2
        * sinf(HBALL_BEAM_MISALIGNMENT_RAD);
    const float effective_accel =
        input->longitudinal_accel_mps2 - centripetal + lateral;

    return atan2f(effective_accel, HBALL_GRAVITY_MPS2)
        - input->body_pitch_rad
        - controller->state[2] / (
            HBALL_ROLLING_FACTOR * HBALL_GRAVITY_MPS2
        );
}

float hball_deployment_controller_command(
    hball_deployment_controller_t *controller,
    const hball_deployment_input_t *input,
    float target_position_m,
    float dt_s,
    bool tracking_enabled
)
{
    float position_error;
    float requested;
    float angle_limit = HBALL_NORMAL_ANGLE_RAD;
    float maximum_change;
    float predicted_position;
    bool edge_recovery = false;

    if ((controller == NULL) || (input == NULL)
        || !isfinite(target_position_m) || !isfinite(dt_s)
        || (dt_s <= 0.0F))
    {
        return 0.0F;
    }
    position_error = controller->state[0] - target_position_m;
    if (tracking_enabled)
    {
        controller->integral_error_m_s = hball_clampf(
            controller->integral_error_m_s + position_error * dt_s,
            -HBALL_INTEGRAL_LIMIT_M_S,
            HBALL_INTEGRAL_LIMIT_M_S
        );
        requested = hball_feedforward(controller, input)
            - g_hball_lqi_position_gain * position_error
            - g_hball_lqi_velocity_gain * controller->state[1]
            - g_hball_lqi_integral_gain
                * controller->integral_error_m_s;
    }
    else
    {
        controller->integral_error_m_s *= 0.98F;
        requested = 0.0F;
    }

    predicted_position = controller->state[0];
    if ((controller->state[0] * controller->state[1]) > 0.0F)
    {
        const float recovery_accel =
            HBALL_ROLLING_FACTOR * HBALL_GRAVITY_MPS2
            * sinf(HBALL_RECOVERY_ANGLE_RAD);
        predicted_position += copysignf(
            controller->state[1] * controller->state[1]
                / (2.0F * recovery_accel),
            controller->state[1]
        );
    }
    if (fabsf(predicted_position)
        >= (HBALL_CENTER_LIMIT_M - HBALL_EDGE_MARGIN_M))
    {
        edge_recovery = true;
        angle_limit = HBALL_RECOVERY_ANGLE_RAD;
        requested = -HBALL_EDGE_POSITION_GAIN * position_error
            - HBALL_EDGE_VELOCITY_GAIN * controller->state[1];
        if (tracking_enabled)
        {
            controller->integral_error_m_s -= position_error * dt_s;
        }
        controller->edge_recovery_total++;
    }
    requested = hball_clampf(requested, -angle_limit, angle_limit);
    requested = hball_clampf(
        requested, -HBALL_HARD_ANGLE_RAD, HBALL_HARD_ANGLE_RAD
    );
    maximum_change = (edge_recovery
        ? HBALL_EDGE_PIPE_RATE_LIMIT_RAD_S
        : HBALL_PIPE_RATE_LIMIT_RAD_S) * dt_s;
    requested = hball_clampf(
        requested,
        controller->previous_pipe_command_rad - maximum_change,
        controller->previous_pipe_command_rad + maximum_change
    );
    if (tracking_enabled && !edge_recovery
        && (fabsf(requested) >= HBALL_NORMAL_ANGLE_RAD))
    {
        controller->integral_error_m_s -= position_error * dt_s;
    }
    controller->previous_pipe_command_rad = requested;
    return requested;
}

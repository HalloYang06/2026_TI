#include "hball_lqg.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define HBALL_GRAVITY_MPS2 9.80665F
#define HBALL_ROLLING_FACTOR (5.0F / 7.0F)
#define HBALL_BEAM_CENTER_OFFSET_M 0.080F
#define HBALL_BEAM_MISALIGNMENT_RAD 0.013962634F
#define HBALL_VISCOUS_DAMPING 0.20F
#define HBALL_CAMERA_NOISE_STD_M 0.0012F
#define HBALL_INNOVATION_GATE_SIGMA 4.0F
#define HBALL_CONTROLLER_DT_S 0.005F
#define HBALL_BEAM_LIMIT_RAD 0.069813170F
#define HBALL_COMMAND_RATE_LIMIT_RAD_S 1.396263402F

static const float hball_lqr_gain[3] = {
    14.56338110F,
    3.31546838F,
    0.58093253F,
};

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

static void hball_matrix_multiply_3x3(
    float left[3][3], float right[3][3], float output[3][3]
)
{
    uint8_t row;
    uint8_t column;
    uint8_t index;

    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            float value = 0.0F;
            for (index = 0U; index < 3U; ++index)
            {
                value += left[row][index] * right[index][column];
            }
            output[row][column] = value;
        }
    }
}

static void hball_covariance_predict(
    hball_lqg_t *controller, float transition[3][3]
)
{
    static const float process_noise[3] = {2.0e-11F, 2.0e-7F, 2.0e-6F};
    float temporary[3][3];
    float transpose[3][3];
    float predicted[3][3];
    uint8_t row;
    uint8_t column;

    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            transpose[row][column] = transition[column][row];
        }
    }
    hball_matrix_multiply_3x3(transition, controller->covariance, temporary);
    hball_matrix_multiply_3x3(temporary, transpose, predicted);
    for (row = 0U; row < 3U; ++row)
    {
        predicted[row][row] += process_noise[row];
    }
    memcpy(controller->covariance, predicted, sizeof(predicted));
}

static void hball_scalar_measurement_update(
    hball_lqg_t *controller,
    uint8_t state_index,
    float innovation,
    float measurement_variance
)
{
    float gain[3];
    float residual[3][3] = {
        {1.0F, 0.0F, 0.0F},
        {0.0F, 1.0F, 0.0F},
        {0.0F, 0.0F, 1.0F},
    };
    float residual_transpose[3][3];
    float temporary[3][3];
    float updated[3][3];
    float innovation_variance;
    uint8_t row;
    uint8_t column;

    innovation_variance = controller->covariance[state_index][state_index]
        + measurement_variance;
    for (row = 0U; row < 3U; ++row)
    {
        gain[row] = controller->covariance[row][state_index] / innovation_variance;
        controller->state[row] += gain[row] * innovation;
        residual[row][state_index] -= gain[row];
    }
    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            residual_transpose[row][column] = residual[column][row];
        }
    }
    hball_matrix_multiply_3x3(residual, controller->covariance, temporary);
    hball_matrix_multiply_3x3(temporary, residual_transpose, updated);
    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            updated[row][column] += gain[row] * measurement_variance * gain[column];
        }
    }
    memcpy(controller->covariance, updated, sizeof(updated));
}

static void hball_velocity_add(
    hball_velocity_estimator_t *estimator, float timestamp_s, float position_m
)
{
    uint8_t index;

    if ((estimator->count > 0U)
        && (timestamp_s <= estimator->timestamp_s[estimator->count - 1U]))
    {
        return;
    }
    if (estimator->count == HBALL_VELOCITY_WINDOW_SIZE)
    {
        for (index = 1U; index < HBALL_VELOCITY_WINDOW_SIZE; ++index)
        {
            estimator->timestamp_s[index - 1U] = estimator->timestamp_s[index];
            estimator->position_m[index - 1U] = estimator->position_m[index];
        }
        estimator->count--;
    }
    estimator->timestamp_s[estimator->count] = timestamp_s;
    estimator->position_m[estimator->count] = position_m;
    estimator->count++;
}

static bool hball_solve_3x3(float matrix[3][4], float solution[3])
{
    uint8_t pivot;
    uint8_t row;
    uint8_t column;

    for (pivot = 0U; pivot < 3U; ++pivot)
    {
        uint8_t best_row = pivot;
        float best_value = fabsf(matrix[pivot][pivot]);
        for (row = (uint8_t)(pivot + 1U); row < 3U; ++row)
        {
            const float candidate = fabsf(matrix[row][pivot]);
            if (candidate > best_value)
            {
                best_value = candidate;
                best_row = row;
            }
        }
        if (best_value < 1.0e-12F)
        {
            return false;
        }
        if (best_row != pivot)
        {
            for (column = pivot; column < 4U; ++column)
            {
                const float temporary = matrix[pivot][column];
                matrix[pivot][column] = matrix[best_row][column];
                matrix[best_row][column] = temporary;
            }
        }
        for (row = (uint8_t)(pivot + 1U); row < 3U; ++row)
        {
            const float factor = matrix[row][pivot] / matrix[pivot][pivot];
            for (column = pivot; column < 4U; ++column)
            {
                matrix[row][column] -= factor * matrix[pivot][column];
            }
        }
    }

    for (row = 3U; row-- > 0U;)
    {
        float value = matrix[row][3];
        for (column = (uint8_t)(row + 1U); column < 3U; ++column)
        {
            value -= matrix[row][column] * solution[column];
        }
        solution[row] = value / matrix[row][row];
    }
    return true;
}

static float hball_velocity_estimate(const hball_velocity_estimator_t *estimator)
{
    float normal[3][4] = {{0.0F}};
    float solution[3] = {0.0F, 0.0F, 0.0F};
    float latest_time;
    uint8_t sample;
    uint8_t row;
    uint8_t column;

    if (estimator->count < 3U)
    {
        return 0.0F;
    }
    latest_time = estimator->timestamp_s[estimator->count - 1U];
    for (sample = 0U; sample < estimator->count; ++sample)
    {
        const float time = estimator->timestamp_s[sample] - latest_time;
        const float basis[3] = {1.0F, time, time * time};
        for (row = 0U; row < 3U; ++row)
        {
            for (column = 0U; column < 3U; ++column)
            {
                normal[row][column] += basis[row] * basis[column];
            }
            normal[row][3] += basis[row] * estimator->position_m[sample];
        }
    }
    return hball_solve_3x3(normal, solution) ? solution[1] : 0.0F;
}

void hball_lqg_init(hball_lqg_t *controller, float initial_position_m)
{
    if (controller == NULL)
    {
        return;
    }
    memset(controller, 0, sizeof(*controller));
    controller->state[0] = initial_position_m;
    controller->covariance[0][0] = 4.0e-6F;
    controller->covariance[1][1] = 4.0e-4F;
    controller->covariance[2][2] = 2.0e-3F;
}

void hball_lqg_predict(
    hball_lqg_t *controller,
    float dt_s,
    const hball_lqg_input_t *input
)
{
    float transition[3][3] = {{0.0F}};
    float world_beam_angle;
    float model_accel;
    float position_gradient;
    float velocity_gradient;
    float position;
    float velocity;

    if ((controller == NULL) || (input == NULL) || (dt_s <= 0.0F))
    {
        return;
    }
    controller->current_time_s += dt_s;
    position = controller->state[0];
    velocity = controller->state[1];
    world_beam_angle = input->beam_angle_rad + input->body_pitch_rad;
    model_accel = HBALL_ROLLING_FACTOR
        * (HBALL_GRAVITY_MPS2 * sinf(world_beam_angle)
            - input->longitudinal_accel_mps2 * cosf(world_beam_angle)
            + input->yaw_rate_rad_s * input->yaw_rate_rad_s
                * (HBALL_BEAM_CENTER_OFFSET_M + position)
            - input->lateral_accel_mps2 * sinf(HBALL_BEAM_MISALIGNMENT_RAD))
        - HBALL_VISCOUS_DAMPING * velocity
        + controller->state[2];
    controller->state[0] = position + velocity * dt_s + 0.5F * model_accel * dt_s * dt_s;
    controller->state[1] = velocity + model_accel * dt_s;
    controller->last_model_accel_mps2 = model_accel;

    position_gradient = HBALL_ROLLING_FACTOR
        * input->yaw_rate_rad_s * input->yaw_rate_rad_s;
    velocity_gradient = -HBALL_VISCOUS_DAMPING;
    transition[0][0] = 1.0F + 0.5F * position_gradient * dt_s * dt_s;
    transition[0][1] = dt_s + 0.5F * velocity_gradient * dt_s * dt_s;
    transition[0][2] = 0.5F * dt_s * dt_s;
    transition[1][0] = position_gradient * dt_s;
    transition[1][1] = 1.0F + velocity_gradient * dt_s;
    transition[1][2] = dt_s;
    transition[2][2] = 1.0F;
    hball_covariance_predict(controller, transition);
}

bool hball_lqg_update_delayed_position(
    hball_lqg_t *controller,
    float measured_position_m,
    float age_s
)
{
    const float position_variance = HBALL_CAMERA_NOISE_STD_M * HBALL_CAMERA_NOISE_STD_M;
    float projected_measurement;
    float innovation;
    float innovation_variance;
    float normalized_innovation_squared;
    float span;

    if ((controller == NULL) || (age_s < 0.0F))
    {
        return false;
    }
    projected_measurement = measured_position_m
        + controller->state[1] * age_s
        + 0.5F * controller->last_model_accel_mps2 * age_s * age_s;
    innovation = projected_measurement - controller->state[0];
    innovation_variance = controller->covariance[0][0] + position_variance;
    if (innovation_variance < 1.0e-12F)
    {
        innovation_variance = 1.0e-12F;
    }
    normalized_innovation_squared = innovation * innovation / innovation_variance;
    if (normalized_innovation_squared
        > HBALL_INNOVATION_GATE_SIGMA * HBALL_INNOVATION_GATE_SIGMA)
    {
        controller->rejected_camera_updates++;
        controller->consecutive_camera_rejections++;
        controller->covariance[0][0] += 4.0F * position_variance;
        return false;
    }

    hball_velocity_add(
        &controller->velocity_estimator,
        controller->current_time_s - age_s,
        measured_position_m
    );
    hball_scalar_measurement_update(controller, 0U, innovation, position_variance);
    span = controller->velocity_estimator.count >= 2U
        ? controller->velocity_estimator.timestamp_s[controller->velocity_estimator.count - 1U]
            - controller->velocity_estimator.timestamp_s[0]
        : 0.0F;
    if (span > 0.0F)
    {
        float velocity_variance = 2.0F * HBALL_CAMERA_NOISE_STD_M / span;
        const float velocity_innovation = hball_velocity_estimate(
            &controller->velocity_estimator
        ) - controller->state[1];
        velocity_variance *= velocity_variance;
        if (velocity_variance < 1.0e-5F)
        {
            velocity_variance = 1.0e-5F;
        }
        hball_scalar_measurement_update(
            controller, 1U, velocity_innovation, velocity_variance
        );
    }
    controller->accepted_camera_updates++;
    controller->consecutive_camera_rejections = 0U;
    return true;
}

static float hball_feedforward(
    const hball_lqg_t *controller, const hball_lqg_input_t *input
)
{
    const float constant = input->yaw_rate_rad_s * input->yaw_rate_rad_s
            * (HBALL_BEAM_CENTER_OFFSET_M + controller->state[0])
        - input->lateral_accel_mps2 * sinf(HBALL_BEAM_MISALIGNMENT_RAD);
    const float amplitude = hypotf(HBALL_GRAVITY_MPS2, input->longitudinal_accel_mps2);
    const float phase = atan2f(-input->longitudinal_accel_mps2, HBALL_GRAVITY_MPS2);
    const float normalized = hball_clampf(-constant / amplitude, -1.0F, 1.0F);

    return asinf(normalized) - phase - input->body_pitch_rad;
}

float hball_lqg_command(
    hball_lqg_t *controller,
    const hball_lqg_input_t *input,
    float target_position_m
)
{
    float feedforward;
    float requested;
    float command;
    const float maximum_step = HBALL_COMMAND_RATE_LIMIT_RAD_S * HBALL_CONTROLLER_DT_S;

    if ((controller == NULL) || (input == NULL))
    {
        return 0.0F;
    }
    feedforward = hball_feedforward(controller, input);
    feedforward -= controller->state[2] / (HBALL_ROLLING_FACTOR * HBALL_GRAVITY_MPS2);
    requested = feedforward
        - hball_lqr_gain[0] * (controller->state[0] - target_position_m)
        - hball_lqr_gain[1] * controller->state[1]
        - hball_lqr_gain[2] * (input->beam_angle_rad - feedforward);
    requested = hball_clampf(requested, -HBALL_BEAM_LIMIT_RAD, HBALL_BEAM_LIMIT_RAD);
    command = hball_clampf(
        requested,
        controller->previous_command_rad - maximum_step,
        controller->previous_command_rad + maximum_step
    );
    controller->previous_command_rad = command;
    return command;
}

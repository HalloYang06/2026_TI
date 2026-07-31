#include "line_follower.h"

#include <stddef.h>
#include <string.h>

enum
{
    TRACK_PHASE_STRAIGHT = 0,
    TRACK_PHASE_CURVE,
    TRACK_PHASE_EXIT_RAMP
};

typedef struct
{
    int16_t base_speed;
    int16_t initial_speed;
    int16_t max_speed;
    int16_t recovery_inner_speed;
    int16_t recovery_outer_speed;
    float position_kp;
    int16_t steering_limit;
    int16_t request_slew_step;
} line_follower_config_t;

static const int16_t MIN_SPEED = 10;
static const uint32_t LOST_TIMEOUT_MS = 700U;

static line_follower_config_t profile_config(line_follower_profile_t profile)
{
    line_follower_config_t config;

    config.base_speed = 50;
    config.initial_speed = 50;
    config.max_speed = 70;
    config.recovery_inner_speed = 14;
    config.recovery_outer_speed = 52;
    config.position_kp = 0.50f;
    config.steering_limit = 20;
    config.request_slew_step = 2;

    if (profile == LINE_FOLLOWER_PROFILE_Q2_FAST_LAP)
    {
        config.base_speed = 63;
        config.initial_speed = 63;
        config.max_speed = 80;
        config.recovery_inner_speed = 24;
        config.recovery_outer_speed = 52;
        config.position_kp = 0.65f;
        config.steering_limit = 28;
        config.request_slew_step = 5;
    }
    else if (profile == LINE_FOLLOWER_PROFILE_STABLE_LAP)
    {
        config.base_speed = 46;
        config.initial_speed = 28;
        config.max_speed = 65;
        config.request_slew_step = 3;
    }

    return config;
}

static int16_t approach_speed(int16_t current, int16_t target, int16_t step)
{
    if (current < target)
    {
        current = (int16_t)(current + step);
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current = (int16_t)(current - step);
        if (current < target)
        {
            current = target;
        }
    }
    return current;
}

static int16_t absolute_error(int16_t error)
{
    return (error < 0) ? (int16_t)(-error) : error;
}

static int16_t stable_ramped_speed(const line_follower_t *follower,
                                   const line_follower_config_t *config,
                                   uint32_t now_ms)
{
    uint32_t elapsed_ms = (uint32_t)(now_ms - follower->run_start_ms);

    if (elapsed_ms >= 1000U)
    {
        return config->base_speed;
    }
    return (int16_t)(
        config->initial_speed +
        (int16_t)(((int32_t)(config->base_speed - config->initial_speed) *
                   (int32_t)elapsed_ms) /
                  1000));
}

static void fill_output(const line_follower_t *follower,
                        const line_snapshot_t *sample,
                        bool reset_wheel_integrators,
                        bool lost_timeout,
                        line_follower_output_t *output)
{
    int16_t duty_slew_step;

    duty_slew_step =
        follower->line_reacquired_pending ?
        ((follower->profile == LINE_FOLLOWER_PROFILE_STABLE_LAP) ? 6 : 15) :
        ((sample->active_count == 0U) ? 10 :
         (((follower->profile == LINE_FOLLOWER_PROFILE_Q2_FAST_LAP) &&
           (follower->last_error_magnitude >= 15)) ? 8 : 3));

    memset(output, 0, sizeof(*output));
    output->intent.valid = !lost_timeout;
    output->intent.timestamp_ms = sample->timestamp_ms;
    output->intent.requested_speed_left = follower->requested_speed_left;
    output->intent.requested_speed_right = follower->requested_speed_right;
    output->intent.duty_slew_step = duty_slew_step;
    output->error = follower->last_error;
    output->reset_wheel_integrators = reset_wheel_integrators;
    output->line_reacquired_pending = follower->line_reacquired_pending;
    output->lost_timeout = lost_timeout;
}

void line_follower_init(line_follower_t *follower,
                        line_follower_profile_t profile,
                        uint32_t run_start_ms)
{
    line_follower_config_t config;

    if (follower == NULL)
    {
        return;
    }
    memset(follower, 0, sizeof(*follower));
    follower->profile = profile;
    follower->run_start_ms = run_start_ms;
    config = profile_config(profile);
    follower->requested_speed_left = config.initial_speed;
    follower->requested_speed_right = config.initial_speed;
}

bool line_follower_step(line_follower_t *follower,
                        const line_snapshot_t *sample,
                        bool force_straight,
                        line_follower_output_t *output)
{
    line_follower_config_t config;
    bool reset_wheel_integrators = false;
    bool lost_timeout = false;
    uint32_t lost_elapsed_ms = 0U;
    int16_t control_base_speed;
    int16_t steering_error;
    int16_t target_steering = 0;
    int16_t desired_speed_left;
    int16_t desired_speed_right;

    if ((follower == NULL) || (sample == NULL) || (output == NULL))
    {
        return false;
    }
    config = profile_config(follower->profile);

    if (force_straight)
    {
        control_base_speed =
            (follower->profile == LINE_FOLLOWER_PROFILE_STABLE_LAP) ?
            stable_ramped_speed(follower, &config, sample->timestamp_ms) :
            config.base_speed;
        follower->requested_speed_left = approach_speed(
            follower->requested_speed_left,
            control_base_speed,
            follower->line_reacquired_pending ? 15 : 2);
        follower->requested_speed_right = approach_speed(
            follower->requested_speed_right,
            control_base_speed,
            follower->line_reacquired_pending ? 15 : 2);
        follower->line_was_lost = false;
        follower->lost_timer_active = false;
    }
    else if (sample->active_count == 0U)
    {
        follower->line_was_lost = true;
        follower->curve_enter_timer_active = false;
        follower->curve_exit_timer_active = false;
        if (follower->profile == LINE_FOLLOWER_PROFILE_Q2_FAST_LAP)
        {
            follower->track_phase = TRACK_PHASE_CURVE;
        }
        if (!follower->lost_timer_active)
        {
            follower->lost_timer_active = true;
            follower->lost_start_ms = sample->timestamp_ms;
        }
        else
        {
            lost_elapsed_ms =
                (uint32_t)(sample->timestamp_ms - follower->lost_start_ms);
        }

        if ((follower->profile != LINE_FOLLOWER_PROFILE_Q2_FAST_LAP) &&
            (lost_elapsed_ms >= LOST_TIMEOUT_MS))
        {
            lost_timeout = true;
        }
        else if ((follower->profile == LINE_FOLLOWER_PROFILE_STABLE_LAP) &&
                 (lost_elapsed_ms < 40U))
        {
            /* Preserve the previous request through a one-sample dropout. */
        }
        else if (follower->profile == LINE_FOLLOWER_PROFILE_STABLE_LAP)
        {
            if (follower->last_line_side < 0)
            {
                follower->requested_speed_left = 20;
                follower->requested_speed_right = 50;
            }
            else if (follower->last_line_side > 0)
            {
                follower->requested_speed_left = 50;
                follower->requested_speed_right = 20;
            }
            else
            {
                follower->requested_speed_left = 36;
                follower->requested_speed_right = 36;
            }
        }
        else if ((follower->profile == LINE_FOLLOWER_PROFILE_Q2_FAST_LAP) &&
                 (lost_elapsed_ms >= 300U))
        {
            if (follower->last_line_side < 0)
            {
                follower->requested_speed_left = 8;
                follower->requested_speed_right = 44;
            }
            else if (follower->last_line_side > 0)
            {
                follower->requested_speed_left = 44;
                follower->requested_speed_right = 8;
            }
            else
            {
                follower->requested_speed_left = 30;
                follower->requested_speed_right = 30;
            }
        }
        else if (follower->last_line_side < 0)
        {
            follower->requested_speed_left = config.recovery_inner_speed;
            follower->requested_speed_right = config.recovery_outer_speed;
        }
        else if (follower->last_line_side > 0)
        {
            follower->requested_speed_left = config.recovery_outer_speed;
            follower->requested_speed_right = config.recovery_inner_speed;
        }
        else
        {
            follower->requested_speed_left = 42;
            follower->requested_speed_right = 42;
        }
    }
    else
    {
        if (follower->line_was_lost)
        {
            follower->line_reacquired_pending = true;
            follower->line_was_lost = false;
            reset_wheel_integrators = true;
        }
        follower->lost_timer_active = false;
        follower->last_error = sample->weighted_error;
        follower->last_error_magnitude = absolute_error(follower->last_error);
        steering_error = follower->last_error;
        control_base_speed = config.base_speed;

        if (follower->profile == LINE_FOLLOWER_PROFILE_Q2_FAST_LAP)
        {
            if (follower->track_phase == TRACK_PHASE_STRAIGHT)
            {
                if (follower->last_error_magnitude >= 15)
                {
                    if (!follower->curve_enter_timer_active)
                    {
                        follower->curve_enter_timer_active = true;
                        follower->curve_enter_start_ms = sample->timestamp_ms;
                    }
                    else if ((uint32_t)(sample->timestamp_ms -
                                        follower->curve_enter_start_ms) >= 20U)
                    {
                        follower->track_phase = TRACK_PHASE_CURVE;
                        follower->curve_enter_timer_active = false;
                        follower->curve_exit_timer_active = false;
                    }
                }
                else
                {
                    follower->curve_enter_timer_active = false;
                }
            }

            if (follower->track_phase == TRACK_PHASE_CURVE)
            {
                control_base_speed = 55;
                if (follower->last_error_magnitude <= 6)
                {
                    if (!follower->curve_exit_timer_active)
                    {
                        follower->curve_exit_timer_active = true;
                        follower->curve_exit_start_ms = sample->timestamp_ms;
                    }
                    else if ((uint32_t)(sample->timestamp_ms -
                                        follower->curve_exit_start_ms) >= 80U)
                    {
                        follower->track_phase = TRACK_PHASE_EXIT_RAMP;
                        follower->curve_ramp_start_ms = sample->timestamp_ms;
                        follower->curve_exit_timer_active = false;
                    }
                }
                else
                {
                    follower->curve_exit_timer_active = false;
                }
            }
            else if (follower->track_phase == TRACK_PHASE_EXIT_RAMP)
            {
                uint32_t ramp_elapsed_ms =
                    (uint32_t)(sample->timestamp_ms -
                               follower->curve_ramp_start_ms);

                if (follower->last_error_magnitude >= 15)
                {
                    follower->track_phase = TRACK_PHASE_CURVE;
                    control_base_speed = 55;
                    follower->curve_exit_timer_active = false;
                }
                else if (ramp_elapsed_ms >= 250U)
                {
                    follower->track_phase = TRACK_PHASE_STRAIGHT;
                    control_base_speed = config.base_speed;
                }
                else
                {
                    control_base_speed =
                        55 +
                        (int16_t)(((int32_t)(config.base_speed - 55) *
                                   (int32_t)ramp_elapsed_ms) /
                                  250);
                }
            }
        }
        else if (follower->profile == LINE_FOLLOWER_PROFILE_STABLE_LAP)
        {
            int16_t stable_curve_feedforward;
            int16_t stable_curve_floor;
            int16_t ramped_base_speed = stable_ramped_speed(
                follower, &config, sample->timestamp_ms);

            if (!follower->stable_filter_ready ||
                follower->line_reacquired_pending)
            {
                follower->stable_filtered_error = follower->last_error;
                follower->stable_filter_ready = true;
            }
            else
            {
                follower->stable_filtered_error =
                    (int16_t)(((int32_t)follower->stable_filtered_error * 3 +
                               (int32_t)follower->last_error * 2) /
                              5);
            }
            steering_error = follower->stable_filtered_error;
            follower->last_error_magnitude = absolute_error(steering_error);
            if (follower->last_error_magnitude <= 5)
            {
                steering_error = 0;
            }

            stable_curve_feedforward =
                (int16_t)(((int32_t)(follower->last_error -
                                     follower->stable_filtered_error) * 3) /
                          10);
            if (stable_curve_feedforward > 4)
            {
                stable_curve_feedforward = 4;
            }
            if (stable_curve_feedforward < -4)
            {
                stable_curve_feedforward = -4;
            }

            control_base_speed =
                ramped_base_speed - follower->last_error_magnitude / 4;
            stable_curve_floor = (ramped_base_speed < 40) ?
                ramped_base_speed : 40;
            if (control_base_speed < stable_curve_floor)
            {
                control_base_speed = stable_curve_floor;
            }
            target_steering =
                (int16_t)(config.position_kp * (float)steering_error) +
                stable_curve_feedforward;
        }

        if (follower->profile != LINE_FOLLOWER_PROFILE_STABLE_LAP)
        {
            target_steering =
                (int16_t)(config.position_kp * (float)steering_error);
        }
        if (target_steering > config.steering_limit)
        {
            target_steering = config.steering_limit;
        }
        if (target_steering < -config.steering_limit)
        {
            target_steering = (int16_t)(-config.steering_limit);
        }

        if (follower->profile == LINE_FOLLOWER_PROFILE_STABLE_LAP)
        {
            follower->stable_steering_command = approach_speed(
                follower->stable_steering_command,
                target_steering,
                follower->line_reacquired_pending ? 4 : 2);
            target_steering = follower->stable_steering_command;
        }

        desired_speed_left = control_base_speed + target_steering;
        desired_speed_right = control_base_speed - target_steering;
        if (desired_speed_left < MIN_SPEED)
        {
            desired_speed_left = MIN_SPEED;
        }
        if (desired_speed_left > config.max_speed)
        {
            desired_speed_left = config.max_speed;
        }
        if (desired_speed_right < MIN_SPEED)
        {
            desired_speed_right = MIN_SPEED;
        }
        if (desired_speed_right > config.max_speed)
        {
            desired_speed_right = config.max_speed;
        }

        follower->requested_speed_left = approach_speed(
            follower->requested_speed_left,
            desired_speed_left,
            follower->line_reacquired_pending ?
            ((follower->profile == LINE_FOLLOWER_PROFILE_STABLE_LAP) ? 5 : 15) :
            config.request_slew_step);
        follower->requested_speed_right = approach_speed(
            follower->requested_speed_right,
            desired_speed_right,
            follower->line_reacquired_pending ?
            ((follower->profile == LINE_FOLLOWER_PROFILE_STABLE_LAP) ? 5 : 15) :
            config.request_slew_step);

        if (!((follower->profile == LINE_FOLLOWER_PROFILE_STABLE_LAP) &&
              follower->line_reacquired_pending))
        {
            if (follower->last_error < -3)
            {
                follower->last_line_side = -1;
            }
            else if (follower->last_error > 3)
            {
                follower->last_line_side = 1;
            }
        }
    }

    fill_output(follower, sample, reset_wheel_integrators,
                lost_timeout, output);
    return true;
}

void line_follower_ack_motion_applied(line_follower_t *follower)
{
    if (follower != NULL)
    {
        follower->line_reacquired_pending = false;
    }
}

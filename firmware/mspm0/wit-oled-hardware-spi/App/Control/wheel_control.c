#include "wheel_control.h"

#include <stddef.h>
#include <string.h>

static int16_t approach_duty(int16_t current, int16_t target, int16_t step)
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

static void initialize_pid(PID_t *pid)
{
    pid->Kp = 0.18f;
    pid->Ki = 0.005f;
    pid->Kd = 0.0f;
    pid->OutMin = -10.0f;
    pid->OutMax = 10.0f;
}

static void clamp_integrator(PID_t *pid)
{
    if (pid->ErrorInt > 80.0f)
    {
        pid->ErrorInt = 80.0f;
    }
    if (pid->ErrorInt < -80.0f)
    {
        pid->ErrorInt = -80.0f;
    }
}

static int16_t clamp_duty(int32_t duty)
{
    if (duty < 6)
    {
        return 6;
    }
    if (duty > 50)
    {
        return 50;
    }
    return (int16_t)duty;
}

void wheel_control_init(
    wheel_control_t *control,
    uint32_t now_ms,
    int16_t initial_duty_left,
    int16_t initial_duty_right
)
{
    if (control == NULL)
    {
        return;
    }
    memset(control, 0, sizeof(*control));
    initialize_pid(&control->left_pid);
    initialize_pid(&control->right_pid);
    control->commanded_duty_left = initial_duty_left;
    control->commanded_duty_right = initial_duty_right;
    control->last_update_ms = now_ms;
}

bool wheel_control_due(const wheel_control_t *control, uint32_t now_ms)
{
    return (control != NULL)
        && ((uint32_t)(now_ms - control->last_update_ms)
            >= WHEEL_CONTROL_PERIOD_MS);
}

bool wheel_control_step(
    wheel_control_t *control,
    uint32_t now_ms,
    int32_t encoder_left_count,
    int32_t encoder_right_count,
    const motion_intent_t *intent,
    wheel_control_output_t *output
)
{
    uint32_t elapsed_ms;
    int32_t measured_left_speed;
    int32_t measured_right_speed;
    int16_t target_duty_left;
    int16_t target_duty_right;

    if ((control == NULL)
        || (intent == NULL)
        || !intent->valid
        || (output == NULL)
        || !wheel_control_due(control, now_ms))
    {
        return false;
    }

    elapsed_ms = (uint32_t)(now_ms - control->last_update_ms);
    measured_left_speed =
        ((encoder_left_count - control->previous_left_count) * 100)
        / (int32_t)elapsed_ms;
    measured_right_speed =
        ((encoder_right_count - control->previous_right_count) * 100)
        / (int32_t)elapsed_ms;
    control->previous_left_count = encoder_left_count;
    control->previous_right_count = encoder_right_count;

    control->left_pid.Target = (float)intent->requested_speed_left;
    control->left_pid.Actual = (float)measured_left_speed;
    control->right_pid.Target = (float)intent->requested_speed_right;
    control->right_pid.Actual = (float)measured_right_speed;
    PID_Update(&control->left_pid);
    PID_Update(&control->right_pid);
    clamp_integrator(&control->left_pid);
    clamp_integrator(&control->right_pid);

    if (intent->requested_speed_left == 0)
    {
        target_duty_left = 0;
        control->left_pid.ErrorInt = 0.0F;
    }
    else
    {
        target_duty_left = clamp_duty(
            ((int32_t)intent->requested_speed_left * 12) / 25
            + 2
            + (int16_t)control->left_pid.Out
        );
    }
    if (intent->requested_speed_right == 0)
    {
        target_duty_right = 0;
        control->right_pid.ErrorInt = 0.0F;
    }
    else
    {
        target_duty_right = clamp_duty(
            ((int32_t)intent->requested_speed_right * 9) / 20
            + 2
            + (int16_t)control->right_pid.Out
        );
    }
    control->commanded_duty_left = approach_duty(
        control->commanded_duty_left,
        target_duty_left,
        intent->duty_slew_step
    );
    control->commanded_duty_right = approach_duty(
        control->commanded_duty_right,
        target_duty_right,
        intent->duty_slew_step
    );
    control->last_update_ms = now_ms;

    output->measured_left_speed = measured_left_speed;
    output->measured_right_speed = measured_right_speed;
    output->duty_left = control->commanded_duty_left;
    output->duty_right = control->commanded_duty_right;
    return true;
}

void wheel_control_reset_integrators(wheel_control_t *control)
{
    if (control == NULL)
    {
        return;
    }
    control->left_pid.ErrorInt = 0.0f;
    control->right_pid.ErrorInt = 0.0f;
}

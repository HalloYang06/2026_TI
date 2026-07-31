#include "wheel_control.h"
#include "motion_intent.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>

typedef struct
{
    PID_t left;
    PID_t right;
    int32_t previous_left_count;
    int32_t previous_right_count;
    int16_t commanded_left;
    int16_t commanded_right;
    uint32_t last_update_ms;
} reference_control_t;

static motion_intent_t make_drive_intent(
    uint32_t timestamp_ms,
    int16_t requested_left,
    int16_t requested_right,
    int16_t slew
)
{
    motion_intent_t intent = {
        true,
        timestamp_ms,
        requested_left,
        requested_right,
        slew,
    };
    return intent;
}

static int16_t reference_approach(
    int16_t current, int16_t target, int16_t step
)
{
    if (current < target)
    {
        current = (int16_t)(current + step);
        if (current > target) current = target;
    }
    else if (current > target)
    {
        current = (int16_t)(current - step);
        if (current < target) current = target;
    }
    return current;
}

static int16_t reference_clamp_duty(int16_t duty)
{
    if (duty < 6) duty = 6;
    if (duty > 50) duty = 50;
    return duty;
}

static void reference_init(reference_control_t *control)
{
    memset(control, 0, sizeof(*control));
    control->left.Kp = 0.18f;
    control->left.Ki = 0.005f;
    control->left.OutMin = -10.0f;
    control->left.OutMax = 10.0f;
    control->right.Kp = 0.18f;
    control->right.Ki = 0.005f;
    control->right.OutMin = -10.0f;
    control->right.OutMax = 10.0f;
    control->commanded_left = 15;
    control->commanded_right = 15;
}

static void reference_step(
    reference_control_t *control,
    uint32_t now_ms,
    int32_t encoder_left_count,
    int32_t encoder_right_count,
    int16_t requested_left,
    int16_t requested_right,
    int16_t slew,
    wheel_control_output_t *output
)
{
    uint32_t elapsed_ms = now_ms - control->last_update_ms;
    int16_t target_left;
    int16_t target_right;

    output->measured_left_speed =
        ((encoder_left_count - control->previous_left_count) * 100)
        / (int32_t)elapsed_ms;
    output->measured_right_speed =
        ((encoder_right_count - control->previous_right_count) * 100)
        / (int32_t)elapsed_ms;
    control->previous_left_count = encoder_left_count;
    control->previous_right_count = encoder_right_count;

    control->left.Target = (float)requested_left;
    control->left.Actual = (float)output->measured_left_speed;
    control->right.Target = (float)requested_right;
    control->right.Actual = (float)output->measured_right_speed;
    PID_Update(&control->left);
    PID_Update(&control->right);
    if (control->left.ErrorInt > 80.0f) control->left.ErrorInt = 80.0f;
    if (control->left.ErrorInt < -80.0f) control->left.ErrorInt = -80.0f;
    if (control->right.ErrorInt > 80.0f) control->right.ErrorInt = 80.0f;
    if (control->right.ErrorInt < -80.0f) control->right.ErrorInt = -80.0f;

    target_left = reference_clamp_duty(
        (int16_t)(((int32_t)requested_left * 12) / 25
                  + 2 + (int16_t)control->left.Out)
    );
    target_right = reference_clamp_duty(
        (int16_t)(((int32_t)requested_right * 9) / 20
                  + 2 + (int16_t)control->right.Out)
    );
    control->commanded_left = reference_approach(
        control->commanded_left, target_left, slew
    );
    control->commanded_right = reference_approach(
        control->commanded_right, target_right, slew
    );
    control->last_update_ms = now_ms;
    output->duty_left = control->commanded_left;
    output->duty_right = control->commanded_right;
}

static void test_update_period_and_first_competition_output(void)
{
    wheel_control_t control;
    wheel_control_output_t output;
    motion_intent_t intent = make_drive_intent(1000U, 63, 63, 3);

    wheel_control_init(&control, 1000U, 15, 15);
    assert(!wheel_control_due(&control, 1099U));
    assert(wheel_control_due(&control, 1100U));
    assert(!wheel_control_step(
        &control, 1099U, 60, 60, &intent, &output));

    assert(wheel_control_step(
        &control, 1100U, 60, 60, &intent, &output));
    assert(output.measured_left_speed == 60);
    assert(output.measured_right_speed == 60);
    assert(output.duty_left == 18);
    assert(output.duty_right == 18);
}

static void test_delayed_sample_is_normalized_to_counts_per_100_ms(void)
{
    wheel_control_t control;
    wheel_control_output_t output;
    motion_intent_t intent = make_drive_intent(150U, 60, 60, 20);

    wheel_control_init(&control, 0U, 15, 15);
    assert(wheel_control_step(
        &control, 150U, 90, 90, &intent, &output));
    assert(output.measured_left_speed == 60);
    assert(output.measured_right_speed == 60);
    assert(output.duty_left == 30);
    assert(output.duty_right == 29);
}

static void test_integrators_are_bounded_and_can_be_reset_on_reacquire(void)
{
    wheel_control_t control;
    wheel_control_output_t output;
    motion_intent_t intent = make_drive_intent(100U, 80, 80, 20);

    wheel_control_init(&control, 0U, 15, 15);
    assert(wheel_control_step(
        &control, 100U, 0, 0, &intent, &output));
    assert(control.left_pid.ErrorInt == 80.0f);
    assert(control.right_pid.ErrorInt == 80.0f);
    assert(output.duty_left == 35);
    assert(output.duty_right == 35);

    intent.timestamp_ms = 200U;
    assert(wheel_control_step(
        &control, 200U, 0, 0, &intent, &output));
    assert(control.left_pid.ErrorInt == 80.0f);
    assert(control.right_pid.ErrorInt == 80.0f);

    wheel_control_reset_integrators(&control);
    assert(control.left_pid.ErrorInt == 0.0f);
    assert(control.right_pid.ErrorInt == 0.0f);
}

static void test_period_check_handles_millisecond_counter_wrap(void)
{
    wheel_control_t control;

    wheel_control_init(&control, UINT32_MAX - 49U, 15, 15);
    assert(!wheel_control_due(&control, 49U));
    assert(wheel_control_due(&control, 50U));
}

static void test_invalid_motion_intent_does_not_advance_control_state(void)
{
    wheel_control_t control;
    wheel_control_output_t output;
    motion_intent_t intent = make_drive_intent(100U, 63, 63, 3);

    wheel_control_init(&control, 0U, 15, 15);
    intent.valid = false;
    assert(!wheel_control_step(
        &control, 100U, 60, 60, &intent, &output));
    assert(control.last_update_ms == 0U);
    assert(control.previous_left_count == 0);
    assert(control.previous_right_count == 0);
}

static void test_sequence_matches_the_previous_lap_controller_math(void)
{
    wheel_control_t control;
    reference_control_t reference;
    wheel_control_output_t actual;
    wheel_control_output_t expected;
    uint32_t now_ms = 0U;
    int32_t encoder_left = 0;
    int32_t encoder_right = 0;
    uint8_t index;
    static const int16_t slew_steps[] = {3, 6, 8, 10, 15};

    wheel_control_init(&control, 0U, 15, 15);
    reference_init(&reference);
    for (index = 0U; index < 64U; ++index)
    {
        int16_t requested_left = (int16_t)(10 + ((index * 11U) % 71U));
        int16_t requested_right = (int16_t)(10 + ((index * 13U) % 71U));
        int16_t slew = slew_steps[index % 5U];
        motion_intent_t intent;

        now_ms += (uint32_t)(100U + ((index * 7U) % 38U));
        encoder_left += (int32_t)((index * 17U) % 91U);
        encoder_right += (int32_t)((index * 19U) % 87U);
        if ((index != 0U) && ((index % 9U) == 0U))
        {
            wheel_control_reset_integrators(&control);
            reference.left.ErrorInt = 0.0f;
            reference.right.ErrorInt = 0.0f;
        }

        reference_step(
            &reference, now_ms, encoder_left, encoder_right,
            requested_left, requested_right, slew, &expected
        );
        intent = make_drive_intent(
            now_ms, requested_left, requested_right, slew
        );
        assert(wheel_control_step(
            &control, now_ms, encoder_left, encoder_right,
            &intent, &actual
        ));
        assert(actual.measured_left_speed == expected.measured_left_speed);
        assert(actual.measured_right_speed == expected.measured_right_speed);
        assert(actual.duty_left == expected.duty_left);
        assert(actual.duty_right == expected.duty_right);
        assert(control.left_pid.ErrorInt == reference.left.ErrorInt);
        assert(control.right_pid.ErrorInt == reference.right.ErrorInt);
    }
}

int main(void)
{
    test_update_period_and_first_competition_output();
    test_delayed_sample_is_normalized_to_counts_per_100_ms();
    test_integrators_are_bounded_and_can_be_reset_on_reacquire();
    test_period_check_handles_millisecond_counter_wrap();
    test_invalid_motion_intent_does_not_advance_control_state();
    test_sequence_matches_the_previous_lap_controller_math();
    return 0;
}

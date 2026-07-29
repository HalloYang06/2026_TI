#include "hball_control_guard.h"

#include <assert.h>
#include <math.h>
#include <string.h>

static hball_sensor_snapshot_t make_sensor(uint32_t sequence)
{
    hball_sensor_snapshot_t sensor;

    memset(&sensor, 0, sizeof(sensor));
    sensor.sequence = sequence;
    sensor.valid_flags = HBALL_SENSOR_VALID_VISION
        | HBALL_SENSOR_VALID_IMU
        | HBALL_SENSOR_VALID_MOTOR
        | HBALL_SENSOR_VALID_HEARTBEAT;
    sensor.vision_confidence = 0.9F;
    sensor.vision_receive_age_ms = 5U;
    sensor.imu_age_ms = 4U;
    sensor.motor_age_ms = 4U;
    sensor.heartbeat_age_ms = 10U;
    return sensor;
}

static hball_control_shadow_t make_shadow(
    uint32_t step, uint32_t source_sequence, float target_rad
)
{
    hball_control_shadow_t shadow;

    memset(&shadow, 0, sizeof(shadow));
    shadow.source_sensor_sequence = source_sequence;
    shadow.controller_steps = step;
    shadow.mode = 1U;
    shadow.flags = HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY
        | HBALL_IPC_CONTROL_FLAG_SAFETY_ELIGIBLE;
    shadow.target_angle_rad = target_rad;
    shadow.estimated_position_m = 0.01F;
    shadow.estimated_velocity_mps = 0.02F;
    shadow.estimated_disturbance_mps2 = 0.03F;
    return shadow;
}

static void test_valid_shadow_is_qualified_but_never_authorizes_tx(void)
{
    hball_control_guard_t guard;
    hball_control_decision_t decision;
    hball_sensor_snapshot_t sensor = make_sensor(100U);
    hball_control_shadow_t shadow = make_shadow(20U, 100U, 0.01F);

    hball_control_guard_init(&guard);
    assert(hball_control_guard_observe(
        &guard, &shadow, &sensor, 1000U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_OK);
    assert(decision.safety_qualified);
    assert(!decision.actuator_tx_allowed);
    assert(HBALL_CONTROL_GUARD_ACTUATOR_TX_ENABLED == 0U);
    assert(hball_control_guard_is_fresh(&guard, 1020U));
    assert(!hball_control_guard_is_fresh(&guard, 1021U));
}

static void test_duplicate_out_of_order_and_source_lag_are_rejected(void)
{
    hball_control_guard_t guard;
    hball_control_decision_t decision;
    hball_sensor_snapshot_t sensor = make_sensor(50U);
    hball_control_shadow_t shadow = make_shadow(10U, 50U, 0.0F);

    hball_control_guard_init(&guard);
    assert(hball_control_guard_observe(
        &guard, &shadow, &sensor, 100U, &decision
    ));
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 101U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_DUPLICATE);

    shadow.controller_steps = 9U;
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 102U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_OUT_OF_ORDER);

    shadow = make_shadow(11U, 45U, 0.0F);
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 105U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_SOURCE_LAG);

    shadow = make_shadow(12U, 51U, 0.0F);
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 110U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_SOURCE_AHEAD);
}

static void test_sensor_interlocks_and_control_flags_fail_closed(void)
{
    hball_control_guard_t guard;
    hball_control_decision_t decision;
    hball_sensor_snapshot_t sensor = make_sensor(8U);
    hball_control_shadow_t shadow = make_shadow(1U, 8U, 0.0F);

    hball_control_guard_init(&guard);
    shadow.flags = HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY;
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 1U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_NOT_ELIGIBLE);

    hball_control_guard_init(&guard);
    shadow = make_shadow(1U, 8U, 0.0F);
    sensor.valid_flags |= HBALL_SENSOR_ESTOP_ACTIVE;
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 1U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_ESTOP);

    hball_control_guard_init(&guard);
    sensor = make_sensor(8U);
    sensor.motor_fault_summary = 1U;
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 1U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_MOTOR_FAULT);

    hball_control_guard_init(&guard);
    sensor = make_sensor(8U);
    sensor.valid_flags &= ~HBALL_SENSOR_VALID_IMU;
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 1U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_SENSOR_INVALID);
}

static void test_angle_finite_and_slew_limits_fail_closed(void)
{
    hball_control_guard_t guard;
    hball_control_decision_t decision;
    hball_sensor_snapshot_t sensor = make_sensor(9U);
    hball_control_shadow_t shadow = make_shadow(1U, 9U, 0.01F);

    hball_control_guard_init(&guard);
    assert(hball_control_guard_observe(
        &guard, &shadow, &sensor, 100U, &decision
    ));

    shadow = make_shadow(2U, 9U, 0.03F);
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 105U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_SLEW_LIMIT);

    hball_control_guard_init(&guard);
    shadow = make_shadow(1U, 9U, 0.13F);
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 100U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_ANGLE_LIMIT);

    hball_control_guard_init(&guard);
    shadow = make_shadow(1U, 9U, NAN);
    assert(!hball_control_guard_observe(
        &guard, &shadow, &sensor, 100U, &decision
    ));
    assert(decision.reason == HBALL_CONTROL_GUARD_NONFINITE);
}

int main(void)
{
    test_valid_shadow_is_qualified_but_never_authorizes_tx();
    test_duplicate_out_of_order_and_source_lag_are_rejected();
    test_sensor_interlocks_and_control_flags_fail_closed();
    test_angle_finite_and_slew_limits_fail_closed();
    return 0;
}

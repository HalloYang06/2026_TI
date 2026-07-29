#include "hball_control_guard.h"

#include "hball_control_pipeline.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool hball_control_guard_reject(
    hball_control_guard_t *guard,
    hball_control_decision_t *decision,
    hball_control_guard_reason_t reason
)
{
    guard->last_reason = reason;
    guard->rejected_total++;
    decision->reason = reason;
    return false;
}

void hball_control_guard_init(hball_control_guard_t *guard)
{
    if (guard != NULL)
    {
        memset(guard, 0, sizeof(*guard));
        guard->last_reason = HBALL_CONTROL_GUARD_ARGUMENT;
    }
}

static bool hball_control_guard_values_are_finite(
    const hball_control_shadow_t *shadow,
    const hball_sensor_snapshot_t *sensor
)
{
    return isfinite(shadow->target_angle_rad)
        && isfinite(shadow->estimated_position_m)
        && isfinite(shadow->estimated_velocity_mps)
        && isfinite(shadow->estimated_disturbance_mps2)
        && isfinite(sensor->vision_confidence);
}

static hball_control_guard_reason_t hball_control_guard_sensor_reason(
    const hball_sensor_snapshot_t *sensor
)
{
    const uint32_t required = HBALL_SENSOR_VALID_VISION
        | HBALL_SENSOR_VALID_IMU
        | HBALL_SENSOR_VALID_MOTOR
        | HBALL_SENSOR_VALID_HEARTBEAT;

    if ((sensor->valid_flags & HBALL_SENSOR_ESTOP_ACTIVE) != 0U)
    {
        return HBALL_CONTROL_GUARD_ESTOP;
    }
    if (sensor->motor_fault_summary != 0U)
    {
        return HBALL_CONTROL_GUARD_MOTOR_FAULT;
    }
    if ((sensor->valid_flags & required) != required)
    {
        return HBALL_CONTROL_GUARD_SENSOR_INVALID;
    }
    if ((sensor->vision_receive_age_ms > HBALL_CONTROL_TRACKING_MAX_AGE_MS)
        || (sensor->imu_age_ms > HBALL_SENSOR_IMU_STALE_MS)
        || (sensor->motor_age_ms > HBALL_SENSOR_MOTOR_STALE_MS)
        || (sensor->heartbeat_age_ms > HBALL_SENSOR_HEARTBEAT_STALE_MS))
    {
        return HBALL_CONTROL_GUARD_SENSOR_STALE;
    }
    if (sensor->vision_confidence < HBALL_CONTROL_MIN_VISION_CONFIDENCE)
    {
        return HBALL_CONTROL_GUARD_LOW_CONFIDENCE;
    }
    return HBALL_CONTROL_GUARD_OK;
}

bool hball_control_guard_observe(
    hball_control_guard_t *guard,
    const hball_control_shadow_t *shadow,
    const hball_sensor_snapshot_t *sensor,
    uint32_t now_ms,
    hball_control_decision_t *decision
)
{
    uint32_t delta;
    hball_control_guard_reason_t sensor_reason;

    if ((guard == NULL) || (shadow == NULL) || (sensor == NULL)
        || (decision == NULL))
    {
        return false;
    }
    memset(decision, 0, sizeof(*decision));
    decision->reason = HBALL_CONTROL_GUARD_ARGUMENT;
    decision->shadow_target_rad = shadow->target_angle_rad;

    if (!hball_control_guard_values_are_finite(shadow, sensor))
    {
        return hball_control_guard_reject(
            guard, decision, HBALL_CONTROL_GUARD_NONFINITE
        );
    }
    if ((shadow->flags & HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY) == 0U)
    {
        return hball_control_guard_reject(
            guard, decision, HBALL_CONTROL_GUARD_FLAGS
        );
    }
    if (guard->step_initialized)
    {
        delta = shadow->controller_steps - guard->last_controller_step;
        if (delta == 0U)
        {
            guard->duplicate_total++;
            return hball_control_guard_reject(
                guard, decision, HBALL_CONTROL_GUARD_DUPLICATE
            );
        }
        if (delta >= UINT32_C(0x80000000))
        {
            return hball_control_guard_reject(
                guard, decision, HBALL_CONTROL_GUARD_OUT_OF_ORDER
            );
        }
    }

    delta = sensor->sequence - shadow->source_sensor_sequence;
    if (delta >= UINT32_C(0x80000000))
    {
        return hball_control_guard_reject(
            guard, decision, HBALL_CONTROL_GUARD_SOURCE_AHEAD
        );
    }
    if (delta > HBALL_CONTROL_GUARD_MAX_SOURCE_LAG_STEPS)
    {
        return hball_control_guard_reject(
            guard, decision, HBALL_CONTROL_GUARD_SOURCE_LAG
        );
    }

    guard->last_controller_step = shadow->controller_steps;
    guard->last_observed_ms = now_ms;
    guard->step_initialized = true;

    sensor_reason = hball_control_guard_sensor_reason(sensor);
    if (sensor_reason != HBALL_CONTROL_GUARD_OK)
    {
        return hball_control_guard_reject(guard, decision, sensor_reason);
    }
    if ((shadow->flags & HBALL_IPC_CONTROL_FLAG_SAFETY_ELIGIBLE) == 0U)
    {
        return hball_control_guard_reject(
            guard, decision, HBALL_CONTROL_GUARD_NOT_ELIGIBLE
        );
    }
    if (shadow->mode != (uint16_t)HBALL_CONTROL_TRACKING)
    {
        return hball_control_guard_reject(
            guard, decision, HBALL_CONTROL_GUARD_MODE
        );
    }
    if (fabsf(shadow->target_angle_rad) > HBALL_CONTROL_GUARD_MAX_TARGET_RAD)
    {
        return hball_control_guard_reject(
            guard, decision, HBALL_CONTROL_GUARD_ANGLE_LIMIT
        );
    }
    if (guard->qualified_initialized)
    {
        const uint32_t elapsed_ms = now_ms - guard->last_qualified_ms;
        const float maximum_change = HBALL_CONTROL_GUARD_MAX_SLEW_RAD_S
            * (float)elapsed_ms * 0.001F;

        if (fabsf(shadow->target_angle_rad
                - guard->last_qualified_target_rad) > maximum_change)
        {
            return hball_control_guard_reject(
                guard, decision, HBALL_CONTROL_GUARD_SLEW_LIMIT
            );
        }
    }

    guard->last_qualified_ms = now_ms;
    guard->last_qualified_target_rad = shadow->target_angle_rad;
    guard->qualified_initialized = true;
    guard->qualified_total++;
    guard->last_reason = HBALL_CONTROL_GUARD_OK;
    decision->reason = HBALL_CONTROL_GUARD_OK;
    decision->safety_qualified = true;
    decision->actuator_tx_allowed = false;
    return true;
}

bool hball_control_guard_is_fresh(
    const hball_control_guard_t *guard, uint32_t now_ms
)
{
    return (guard != NULL) && guard->step_initialized
        && ((uint32_t)(now_ms - guard->last_observed_ms)
            <= HBALL_CONTROL_GUARD_MAX_AGE_MS);
}

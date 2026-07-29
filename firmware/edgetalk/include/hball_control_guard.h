#ifndef HBALL_CONTROL_GUARD_H
#define HBALL_CONTROL_GUARD_H

#include "hball_dualcore_ipc.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_CONTROL_GUARD_ACTUATOR_TX_ENABLED 0U
#define HBALL_CONTROL_GUARD_MAX_AGE_MS 20U
#define HBALL_CONTROL_GUARD_MAX_SOURCE_LAG_STEPS 4U
#define HBALL_CONTROL_GUARD_MAX_TARGET_RAD 0.12F
#define HBALL_CONTROL_GUARD_MAX_SLEW_RAD_S 2.0F

typedef enum
{
    HBALL_CONTROL_GUARD_OK = 0,
    HBALL_CONTROL_GUARD_ARGUMENT,
    HBALL_CONTROL_GUARD_NONFINITE,
    HBALL_CONTROL_GUARD_FLAGS,
    HBALL_CONTROL_GUARD_DUPLICATE,
    HBALL_CONTROL_GUARD_OUT_OF_ORDER,
    HBALL_CONTROL_GUARD_SOURCE_AHEAD,
    HBALL_CONTROL_GUARD_SOURCE_LAG,
    HBALL_CONTROL_GUARD_ESTOP,
    HBALL_CONTROL_GUARD_MOTOR_FAULT,
    HBALL_CONTROL_GUARD_SENSOR_INVALID,
    HBALL_CONTROL_GUARD_SENSOR_STALE,
    HBALL_CONTROL_GUARD_LOW_CONFIDENCE,
    HBALL_CONTROL_GUARD_NOT_ELIGIBLE,
    HBALL_CONTROL_GUARD_MODE,
    HBALL_CONTROL_GUARD_ANGLE_LIMIT,
    HBALL_CONTROL_GUARD_SLEW_LIMIT,
} hball_control_guard_reason_t;

typedef struct
{
    hball_control_guard_reason_t reason;
    bool safety_qualified;
    bool actuator_tx_allowed;
    float shadow_target_rad;
} hball_control_decision_t;

typedef struct
{
    uint32_t last_controller_step;
    uint32_t last_observed_ms;
    uint32_t last_qualified_ms;
    uint32_t qualified_total;
    uint32_t rejected_total;
    uint32_t duplicate_total;
    float last_qualified_target_rad;
    hball_control_guard_reason_t last_reason;
    bool step_initialized;
    bool qualified_initialized;
} hball_control_guard_t;

void hball_control_guard_init(hball_control_guard_t *guard);
bool hball_control_guard_observe(
    hball_control_guard_t *guard,
    const hball_control_shadow_t *shadow,
    const hball_sensor_snapshot_t *sensor,
    uint32_t now_ms,
    hball_control_decision_t *decision
);
bool hball_control_guard_is_fresh(
    const hball_control_guard_t *guard, uint32_t now_ms
);

#ifdef __cplusplus
}
#endif

#endif

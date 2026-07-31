#include "hball_rs00_control.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define HBALL_RS00_TYPE_ENABLE 0x03U
#define HBALL_RS00_TYPE_STOP 0x04U
#define HBALL_RS00_TYPE_SET_SINGLE_PARAMETER 0x12U

static bool hball_rs00_control_make_empty(
    uint8_t comm_type, uint8_t motor_id, hball_can_frame_t *frame
)
{
    if ((motor_id == 0U) || (frame == NULL))
    {
        return false;
    }
    memset(frame, 0, sizeof(*frame));
    frame->id = hball_rs00_ext_id(
        comm_type, HBALL_RS00_MASTER_ID, motor_id
    );
    frame->is_extended = 1U;
    frame->dlc = 8U;
    return true;
}

static void hball_rs00_store_float_le(uint8_t *target, float value)
{
    uint32_t bits;

    memcpy(&bits, &value, sizeof(bits));
    target[0] = (uint8_t)bits;
    target[1] = (uint8_t)(bits >> 8U);
    target[2] = (uint8_t)(bits >> 16U);
    target[3] = (uint8_t)(bits >> 24U);
}

static bool hball_rs00_control_make_parameter_prefix(
    uint8_t motor_id, uint16_t index, hball_can_frame_t *frame
)
{
    if (!hball_rs00_control_make_empty(
            HBALL_RS00_TYPE_SET_SINGLE_PARAMETER, motor_id, frame))
    {
        return false;
    }
    frame->data[0] = (uint8_t)index;
    frame->data[1] = (uint8_t)(index >> 8U);
    return true;
}

static bool hball_rs00_control_make_float_parameter(
    uint8_t motor_id,
    uint16_t index,
    float value,
    float minimum,
    float maximum,
    hball_can_frame_t *frame
)
{
    if (!isfinite(value) || (value < minimum) || (value > maximum)
        || !hball_rs00_control_make_parameter_prefix(
            motor_id, index, frame))
    {
        return false;
    }
    hball_rs00_store_float_le(frame->data + 4U, value);
    return true;
}

bool hball_rs00_control_make_stop(
    uint8_t motor_id, bool clear_error, hball_can_frame_t *frame
)
{
    if (!hball_rs00_control_make_empty(
            HBALL_RS00_TYPE_STOP, motor_id, frame))
    {
        return false;
    }
    frame->data[0] = clear_error ? 1U : 0U;
    return true;
}

bool hball_rs00_control_make_enable(
    uint8_t motor_id, hball_can_frame_t *frame
)
{
    return hball_rs00_control_make_empty(
        HBALL_RS00_TYPE_ENABLE, motor_id, frame
    );
}

bool hball_rs00_control_make_csp_mode(
    uint8_t motor_id, hball_can_frame_t *frame
)
{
    if (!hball_rs00_control_make_parameter_prefix(
            motor_id, HBALL_RS00_INDEX_RUN_MODE, frame))
    {
        return false;
    }
    frame->data[4] = HBALL_RS00_CSP_MODE;
    return true;
}

bool hball_rs00_control_make_speed_limit(
    uint8_t motor_id, float speed_rad_s, hball_can_frame_t *frame
)
{
    return hball_rs00_control_make_float_parameter(
        motor_id,
        HBALL_RS00_INDEX_SPEED_LIMIT,
        speed_rad_s,
        0.05F,
        HBALL_RS00_SPEED_LIMIT_MAX_RAD_S,
        frame
    );
}

bool hball_rs00_control_make_current_limit(
    uint8_t motor_id, float current_a, hball_can_frame_t *frame
)
{
    return hball_rs00_control_make_float_parameter(
        motor_id,
        HBALL_RS00_INDEX_CURRENT_LIMIT,
        current_a,
        0.05F,
        2.0F,
        frame
    );
}

bool hball_rs00_control_make_position_kp(
    uint8_t motor_id, float position_kp, hball_can_frame_t *frame
)
{
    return hball_rs00_control_make_float_parameter(
        motor_id,
        HBALL_RS00_INDEX_POSITION_KP,
        position_kp,
        0.0F,
        200.0F,
        frame
    );
}

bool hball_rs00_control_make_position_reference(
    uint8_t motor_id, float position_rad, hball_can_frame_t *frame
)
{
    return hball_rs00_control_make_float_parameter(
        motor_id,
        HBALL_RS00_INDEX_POSITION_REFERENCE,
        position_rad,
        HBALL_RS00_POSITION_MIN_RAD,
        HBALL_RS00_POSITION_MAX_RAD,
        frame
    );
}

static bool hball_rs00_bench_is_active(const hball_rs00_bench_t *bench)
{
    return (bench->state >= HBALL_RS00_BENCH_PREPARING)
        && (bench->state <= HBALL_RS00_BENCH_RETURNING);
}

void hball_rs00_bench_init(hball_rs00_bench_t *bench)
{
    if (bench == NULL)
    {
        return;
    }
    memset(bench, 0, sizeof(*bench));
    bench->state = HBALL_RS00_BENCH_SAFE;
}

bool hball_rs00_bench_begin_prepare(
    hball_rs00_bench_t *bench, float current_position_rad, uint32_t now_ms
)
{
    if ((bench == NULL) || !isfinite(current_position_rad)
        || (current_position_rad < HBALL_RS00_POSITION_MIN_RAD)
        || (current_position_rad > HBALL_RS00_POSITION_MAX_RAD)
        || ((bench->state != HBALL_RS00_BENCH_SAFE)
            && (bench->state != HBALL_RS00_BENCH_STOPPED)))
    {
        return false;
    }
    bench->state = HBALL_RS00_BENCH_PREPARING;
    bench->stop_reason = HBALL_RS00_BENCH_STOP_NONE;
    bench->initial_position_rad = current_position_rad;
    bench->target_position_rad = current_position_rad;
    bench->state_since_ms = now_ms;
    bench->last_manual_command_ms = now_ms;
    bench->arm_request_ms = 0U;
    bench->feedback_seen_after_arm = false;
    return true;
}

bool hball_rs00_bench_mark_prepared(
    hball_rs00_bench_t *bench, uint8_t run_mode, uint32_t now_ms
)
{
    if ((bench == NULL) || (bench->state != HBALL_RS00_BENCH_PREPARING))
    {
        return false;
    }
    if (run_mode != HBALL_RS00_CSP_MODE)
    {
        hball_rs00_bench_trip(
            bench, HBALL_RS00_BENCH_STOP_MODE_REJECTED, now_ms
        );
        return false;
    }
    bench->state = HBALL_RS00_BENCH_PREPARED;
    bench->state_since_ms = now_ms;
    return true;
}

bool hball_rs00_bench_begin_arm(
    hball_rs00_bench_t *bench, uint32_t now_ms
)
{
    if ((bench == NULL) || (bench->state != HBALL_RS00_BENCH_PREPARED))
    {
        return false;
    }
    bench->state = HBALL_RS00_BENCH_ARMING;
    bench->state_since_ms = now_ms;
    bench->last_manual_command_ms = now_ms;
    bench->arm_request_ms = now_ms;
    bench->feedback_seen_after_arm = false;
    return true;
}

bool hball_rs00_bench_accept_feedback(
    hball_rs00_bench_t *bench,
    uint8_t fault_summary,
    uint32_t feedback_ms,
    uint32_t now_ms
)
{
    if ((bench == NULL) || !hball_rs00_bench_is_active(bench))
    {
        return false;
    }
    if (fault_summary != 0U)
    {
        hball_rs00_bench_trip(
            bench, HBALL_RS00_BENCH_STOP_MOTOR_FAULT, now_ms
        );
        return false;
    }
    if ((bench->state == HBALL_RS00_BENCH_ARMING)
        && ((int32_t)(feedback_ms - bench->arm_request_ms) >= 0))
    {
        bench->feedback_seen_after_arm = true;
        bench->state = HBALL_RS00_BENCH_ARMED;
        bench->state_since_ms = now_ms;
    }
    return true;
}

bool hball_rs00_bench_make_step(
    hball_rs00_bench_t *bench,
    float step_rad,
    uint32_t now_ms,
    float *target_position_rad
)
{
    float target;

    if ((bench == NULL) || (target_position_rad == NULL)
        || !isfinite(step_rad)
        || (fabsf(step_rad) > HBALL_RS00_BENCH_STEP_MAX_RAD)
        || (bench->state != HBALL_RS00_BENCH_ARMED))
    {
        return false;
    }
    target = bench->target_position_rad + step_rad;
    if (!isfinite(target)
        || (fabsf(target - bench->initial_position_rad)
            > HBALL_RS00_BENCH_ENVELOPE_RAD)
        || (target < HBALL_RS00_POSITION_MIN_RAD)
        || (target > HBALL_RS00_POSITION_MAX_RAD))
    {
        return false;
    }
    bench->target_position_rad = target;
    bench->state_since_ms = now_ms;
    bench->last_manual_command_ms = now_ms;
    *target_position_rad = target;
    return true;
}

bool hball_rs00_bench_make_return(
    hball_rs00_bench_t *bench,
    uint32_t now_ms,
    float *target_position_rad
)
{
    if ((bench == NULL) || (target_position_rad == NULL)
        || ((bench->state != HBALL_RS00_BENCH_SMALL_STEP)
            && (bench->state != HBALL_RS00_BENCH_ARMED)))
    {
        return false;
    }
    bench->target_position_rad = bench->initial_position_rad;
    bench->state = HBALL_RS00_BENCH_RETURNING;
    bench->state_since_ms = now_ms;
    bench->last_manual_command_ms = now_ms;
    *target_position_rad = bench->target_position_rad;
    return true;
}

bool hball_rs00_bench_watchdog_expired(
    hball_rs00_bench_t *bench, uint32_t now_ms
)
{
    if ((bench == NULL) || !hball_rs00_bench_is_active(bench))
    {
        return false;
    }
    if ((bench->state == HBALL_RS00_BENCH_ARMING)
        && !bench->feedback_seen_after_arm
        && ((uint32_t)(now_ms - bench->arm_request_ms)
            >= HBALL_RS00_BENCH_FEEDBACK_TIMEOUT_MS))
    {
        hball_rs00_bench_trip(
            bench, HBALL_RS00_BENCH_STOP_FEEDBACK_TIMEOUT, now_ms
        );
        return true;
    }
    if ((bench->state != HBALL_RS00_BENCH_ARMED)
        && ((uint32_t)(now_ms - bench->last_manual_command_ms)
            >= HBALL_RS00_BENCH_COMMAND_TIMEOUT_MS))
    {
        hball_rs00_bench_trip(
            bench, HBALL_RS00_BENCH_STOP_COMMAND_TIMEOUT, now_ms
        );
        return true;
    }
    return false;
}

void hball_rs00_bench_trip(
    hball_rs00_bench_t *bench,
    hball_rs00_bench_stop_reason_t reason,
    uint32_t now_ms
)
{
    if (bench == NULL)
    {
        return;
    }
    bench->state = HBALL_RS00_BENCH_FAULT;
    bench->stop_reason = reason;
    bench->state_since_ms = now_ms;
}

void hball_rs00_bench_mark_stopped(
    hball_rs00_bench_t *bench,
    hball_rs00_bench_stop_reason_t reason,
    uint32_t now_ms
)
{
    if (bench == NULL)
    {
        return;
    }
    bench->state = HBALL_RS00_BENCH_STOPPED;
    bench->stop_reason = reason;
    bench->state_since_ms = now_ms;
    bench->feedback_seen_after_arm = false;
}

const char *hball_rs00_bench_state_name(hball_rs00_bench_state_t state)
{
    static const char *const names[] = {
        "SAFE", "PREPARING", "PREPARED", "ARMING", "ARMED",
        "SMALL_STEP", "RETURNING", "STOPPED", "FAULT",
    };

    if ((unsigned int)state >= (sizeof(names) / sizeof(names[0])))
    {
        return "UNKNOWN";
    }
    return names[state];
}

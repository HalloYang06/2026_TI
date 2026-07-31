#ifndef HBALL_RS00_CONTROL_H
#define HBALL_RS00_CONTROL_H

#include "hball_can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_RS00_CSP_MODE 5U
#define HBALL_RS00_INDEX_POSITION_REFERENCE 0x7016U
#define HBALL_RS00_INDEX_SPEED_LIMIT 0x7017U
#define HBALL_RS00_INDEX_CURRENT_LIMIT 0x7018U

#define HBALL_RS00_BENCH_SPEED_LIMIT_RAD_S 1.0F
#define HBALL_RS00_BENCH_CURRENT_LIMIT_A 0.8F
#define HBALL_RS00_BENCH_STEP_MAX_RAD 0.02F
#define HBALL_RS00_BENCH_ENVELOPE_RAD 0.05F
#define HBALL_RS00_BENCH_FEEDBACK_TIMEOUT_MS 500U
#define HBALL_RS00_BENCH_COMMAND_TIMEOUT_MS 2000U

typedef enum
{
    HBALL_RS00_BENCH_SAFE = 0,
    HBALL_RS00_BENCH_PREPARING,
    HBALL_RS00_BENCH_PREPARED,
    HBALL_RS00_BENCH_ARMING,
    HBALL_RS00_BENCH_ARMED,
    HBALL_RS00_BENCH_SMALL_STEP,
    HBALL_RS00_BENCH_RETURNING,
    HBALL_RS00_BENCH_STOPPED,
    HBALL_RS00_BENCH_FAULT,
} hball_rs00_bench_state_t;

typedef enum
{
    HBALL_RS00_BENCH_STOP_NONE = 0,
    HBALL_RS00_BENCH_STOP_MANUAL,
    HBALL_RS00_BENCH_STOP_TX_FAILURE,
    HBALL_RS00_BENCH_STOP_MODE_REJECTED,
    HBALL_RS00_BENCH_STOP_MODE_TIMEOUT,
    HBALL_RS00_BENCH_STOP_FEEDBACK_TIMEOUT,
    HBALL_RS00_BENCH_STOP_COMMAND_TIMEOUT,
    HBALL_RS00_BENCH_STOP_RETURN_TIMEOUT,
    HBALL_RS00_BENCH_STOP_MOTOR_FAULT,
    HBALL_RS00_BENCH_STOP_SENSOR_INVALID,
} hball_rs00_bench_stop_reason_t;

typedef struct
{
    hball_rs00_bench_state_t state;
    hball_rs00_bench_stop_reason_t stop_reason;
    float initial_position_rad;
    float target_position_rad;
    uint32_t state_since_ms;
    uint32_t last_manual_command_ms;
    uint32_t arm_request_ms;
    bool feedback_seen_after_arm;
} hball_rs00_bench_t;

bool hball_rs00_control_make_stop(
    uint8_t motor_id, bool clear_error, hball_can_frame_t *frame
);
bool hball_rs00_control_make_enable(
    uint8_t motor_id, hball_can_frame_t *frame
);
bool hball_rs00_control_make_csp_mode(
    uint8_t motor_id, hball_can_frame_t *frame
);
bool hball_rs00_control_make_speed_limit(
    uint8_t motor_id, float speed_rad_s, hball_can_frame_t *frame
);
bool hball_rs00_control_make_current_limit(
    uint8_t motor_id, float current_a, hball_can_frame_t *frame
);
bool hball_rs00_control_make_position_reference(
    uint8_t motor_id, float position_rad, hball_can_frame_t *frame
);

void hball_rs00_bench_init(hball_rs00_bench_t *bench);
bool hball_rs00_bench_begin_prepare(
    hball_rs00_bench_t *bench, float current_position_rad, uint32_t now_ms
);
bool hball_rs00_bench_mark_prepared(
    hball_rs00_bench_t *bench, uint8_t run_mode, uint32_t now_ms
);
bool hball_rs00_bench_begin_arm(
    hball_rs00_bench_t *bench, uint32_t now_ms
);
bool hball_rs00_bench_accept_feedback(
    hball_rs00_bench_t *bench,
    uint8_t fault_summary,
    uint32_t feedback_ms,
    uint32_t now_ms
);
bool hball_rs00_bench_make_step(
    hball_rs00_bench_t *bench,
    float step_rad,
    uint32_t now_ms,
    float *target_position_rad
);
bool hball_rs00_bench_make_return(
    hball_rs00_bench_t *bench,
    uint32_t now_ms,
    float *target_position_rad
);
bool hball_rs00_bench_watchdog_expired(
    hball_rs00_bench_t *bench, uint32_t now_ms
);
void hball_rs00_bench_trip(
    hball_rs00_bench_t *bench,
    hball_rs00_bench_stop_reason_t reason,
    uint32_t now_ms
);
void hball_rs00_bench_mark_stopped(
    hball_rs00_bench_t *bench,
    hball_rs00_bench_stop_reason_t reason,
    uint32_t now_ms
);
const char *hball_rs00_bench_state_name(hball_rs00_bench_state_t state);

#ifdef __cplusplus
}
#endif

#endif

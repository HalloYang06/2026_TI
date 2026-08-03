#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "line_snapshot.h"
#include "motion_intent.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LINE_FOLLOWER_PROFILE_Q2_FAST_LAP = 0,
    LINE_FOLLOWER_PROFILE_Q4_TIMED_RUN,
    LINE_FOLLOWER_PROFILE_STABLE_LAP
} line_follower_profile_t;

typedef struct
{
    motion_intent_t intent;
    int16_t error;
    bool reset_wheel_integrators;
    bool line_reacquired_pending;
    bool lost_timeout;
} line_follower_output_t;

typedef struct
{
    line_follower_profile_t profile;
    uint32_t run_start_ms;
    uint32_t lost_start_ms;
    uint32_t curve_enter_start_ms;
    uint32_t curve_exit_start_ms;
    uint32_t curve_ramp_start_ms;
    int16_t requested_speed_left;
    int16_t requested_speed_right;
    int16_t last_error;
    int16_t last_error_magnitude;
    int16_t stable_filtered_error;
    int16_t stable_steering_command;
    int8_t last_line_side;
    uint8_t track_phase;
    bool line_was_lost;
    bool line_reacquired_pending;
    bool lost_timer_active;
    bool curve_enter_timer_active;
    bool curve_exit_timer_active;
    bool stable_filter_ready;
} line_follower_t;

void line_follower_init(line_follower_t *follower,
                        line_follower_profile_t profile,
                        uint32_t run_start_ms);
bool line_follower_step(line_follower_t *follower,
                        const line_snapshot_t *sample,
                        bool force_straight,
                        line_follower_output_t *output);
void line_follower_ack_motion_applied(line_follower_t *follower);

#endif

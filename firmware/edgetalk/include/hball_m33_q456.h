#ifndef HBALL_M33_Q456_H
#define HBALL_M33_Q456_H

#include "hball_mission_can.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HBALL_Q456_LATCH_SAMPLE_COUNT 5U
#define HBALL_Q456_TARGET_LIMIT_M 0.080F
#define HBALL_Q4_DEADLINE_MS 8000U
#define HBALL_Q5_DEADLINE_MS 30000U
#define HBALL_Q6_DEADLINE_MS 30000U

typedef enum
{
    HBALL_Q456_OUTCOME_NONE = 0,
    HBALL_Q456_OUTCOME_COMPLETED,
    HBALL_Q456_OUTCOME_DEADLINE,
    HBALL_Q456_OUTCOME_CONTROL_FAULT,
} hball_q456_outcome_t;

typedef enum
{
    HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE = 0,
    HBALL_Q456_ROUTE_WAIT_LEFT_A,
    HBALL_Q456_ROUTE_WAIT_REACQUIRE_A,
    HBALL_Q456_ROUTE_WAIT_STOPPED,
} hball_q456_route_phase_t;

typedef struct
{
    uint16_t epoch;
    uint8_t mission_id;
    uint8_t route_phase;
    uint8_t sample_count;
    uint8_t sample_next;
    uint32_t last_vision_sequence;
    uint32_t running_since_ms;
    float samples_m[HBALL_Q456_LATCH_SAMPLE_COUNT];
    float target_position_m;
    bool context_valid;
    bool vision_sequence_valid;
    bool target_latched;
    bool running;
    bool prepare_applied;
} hball_m33_q456_t;

void hball_m33_q456_init(hball_m33_q456_t *runtime);
bool hball_m33_q456_sync_context(
    hball_m33_q456_t *runtime, uint16_t epoch, uint8_t mission_id
);
bool hball_m33_q456_prepare(hball_m33_q456_t *runtime);
void hball_m33_q456_observe_vision(
    hball_m33_q456_t *runtime,
    uint32_t vision_sequence,
    float ball_position_m,
    bool valid
);
bool hball_m33_q456_start_target(
    hball_m33_q456_t *runtime, float *target_position_m
);
bool hball_m33_q456_mark_running(
    hball_m33_q456_t *runtime, uint32_t now_ms
);
hball_q456_outcome_t hball_m33_q456_step(
    hball_m33_q456_t *runtime,
    uint32_t now_ms,
    uint16_t chassis_epoch,
    uint8_t chassis_event_flags,
    bool control_fault
);
bool hball_m33_q456_is_mission(uint8_t mission_id);

#ifdef __cplusplus
}
#endif

#endif

#include "hball_m33_q456.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

bool hball_m33_q456_is_mission(uint8_t mission_id)
{
    return (mission_id == HBALL_MISSION_Q4_A_TO_B)
        || (mission_id == HBALL_MISSION_Q5_CENTER_LAP)
        || (mission_id == HBALL_MISSION_Q6_HOLD_POSITION_LAP);
}

void hball_m33_q456_init(hball_m33_q456_t *runtime)
{
    if (runtime != NULL)
    {
        memset(runtime, 0, sizeof(*runtime));
    }
}

bool hball_m33_q456_sync_context(
    hball_m33_q456_t *runtime, uint16_t epoch, uint8_t mission_id
)
{
    if ((runtime == NULL) || (epoch == 0U)
        || !hball_m33_q456_is_mission(mission_id))
    {
        return false;
    }
    if (runtime->context_valid
        && (runtime->epoch == epoch)
        && (runtime->mission_id == mission_id))
    {
        return true;
    }
    hball_m33_q456_init(runtime);
    runtime->epoch = epoch;
    runtime->mission_id = mission_id;
    runtime->context_valid = true;
    return true;
}

bool hball_m33_q456_prepare(hball_m33_q456_t *runtime)
{
    if ((runtime == NULL) || !runtime->context_valid)
    {
        return false;
    }
    if (runtime->prepare_applied)
    {
        return true;
    }
    runtime->route_phase = HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE;
    runtime->sample_count = 0U;
    runtime->sample_next = 0U;
    runtime->last_vision_sequence = 0U;
    runtime->running_since_ms = 0U;
    runtime->target_position_m = 0.0F;
    runtime->vision_sequence_valid = false;
    runtime->target_latched = false;
    runtime->running = false;
    runtime->prepare_applied = true;
    return true;
}

void hball_m33_q456_observe_vision(
    hball_m33_q456_t *runtime,
    uint32_t vision_sequence,
    float ball_position_m,
    bool valid
)
{
    if ((runtime == NULL) || !runtime->context_valid || !valid
        || !isfinite(ball_position_m)
        || (fabsf(ball_position_m) > HBALL_Q456_TARGET_LIMIT_M)
        || (runtime->vision_sequence_valid
            && (runtime->last_vision_sequence == vision_sequence)))
    {
        return;
    }
    runtime->last_vision_sequence = vision_sequence;
    runtime->vision_sequence_valid = true;
    runtime->samples_m[runtime->sample_next] = ball_position_m;
    runtime->sample_next = (uint8_t)((runtime->sample_next + 1U)
        % HBALL_Q456_LATCH_SAMPLE_COUNT);
    if (runtime->sample_count < HBALL_Q456_LATCH_SAMPLE_COUNT)
    {
        runtime->sample_count++;
    }
}

static float hball_q456_median(const float *values, uint8_t count)
{
    float sorted[HBALL_Q456_LATCH_SAMPLE_COUNT];
    uint8_t index;

    memcpy(sorted, values, (size_t)count * sizeof(sorted[0]));
    for (index = 1U; index < count; ++index)
    {
        const float value = sorted[index];
        uint8_t position = index;

        while ((position > 0U) && (sorted[position - 1U] > value))
        {
            sorted[position] = sorted[position - 1U];
            position--;
        }
        sorted[position] = value;
    }
    return sorted[count / 2U];
}

bool hball_m33_q456_start_target(
    hball_m33_q456_t *runtime, float *target_position_m
)
{
    if ((runtime == NULL) || (target_position_m == NULL)
        || !runtime->context_valid)
    {
        return false;
    }
    if (!runtime->target_latched)
    {
        if ((runtime->mission_id == HBALL_MISSION_Q4_A_TO_B)
            || (runtime->mission_id == HBALL_MISSION_Q5_CENTER_LAP))
        {
            runtime->target_position_m = 0.0F;
        }
        else if ((runtime->mission_id
                  == HBALL_MISSION_Q6_HOLD_POSITION_LAP)
            && (runtime->sample_count == HBALL_Q456_LATCH_SAMPLE_COUNT))
        {
            runtime->target_position_m = hball_q456_median(
                runtime->samples_m, runtime->sample_count
            );
        }
        else
        {
            return false;
        }
        runtime->target_latched = true;
    }
    *target_position_m = runtime->target_position_m;
    runtime->prepare_applied = false;
    return true;
}

bool hball_m33_q456_mark_running(
    hball_m33_q456_t *runtime, uint32_t now_ms
)
{
    if ((runtime == NULL) || !runtime->context_valid
        || !runtime->target_latched)
    {
        return false;
    }
    if (!runtime->running)
    {
        runtime->running = true;
        runtime->running_since_ms = now_ms;
    }
    runtime->prepare_applied = false;
    return true;
}

static hball_q456_outcome_t hball_m33_q4_step(
    const hball_m33_q456_t *runtime,
    uint32_t now_ms,
    uint8_t chassis_event_flags
)
{
    const uint8_t completed = HBALL_MISSION_CHASSIS_EVENT_STOPPED
        | HBALL_MISSION_CHASSIS_EVENT_DETECTED_B;

    if ((chassis_event_flags & completed) == completed)
    {
        return HBALL_Q456_OUTCOME_COMPLETED;
    }
    return ((uint32_t)(now_ms - runtime->running_since_ms)
            >= HBALL_Q4_DEADLINE_MS)
        ? HBALL_Q456_OUTCOME_DEADLINE
        : HBALL_Q456_OUTCOME_NONE;
}

static hball_q456_outcome_t hball_m33_q5_q6_step(
    hball_m33_q456_t *runtime,
    uint32_t now_ms,
    uint8_t chassis_event_flags
)
{
    const uint8_t chassis_faults = HBALL_MISSION_CHASSIS_EVENT_LINE_LOST
        | HBALL_MISSION_CHASSIS_EVENT_LOCAL_FAULT
        | HBALL_MISSION_CHASSIS_EVENT_INHIBITED;
    const bool control_active = (chassis_event_flags
            & HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE)
        != 0U;
    const uint32_t deadline_ms =
        (runtime->mission_id == HBALL_MISSION_Q5_CENTER_LAP)
            ? HBALL_Q5_DEADLINE_MS
            : HBALL_Q6_DEADLINE_MS;

    if ((runtime->route_phase == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE)
        && !control_active)
    {
        return ((uint32_t)(now_ms - runtime->running_since_ms)
                >= deadline_ms)
            ? HBALL_Q456_OUTCOME_DEADLINE
            : HBALL_Q456_OUTCOME_NONE;
    }
    if ((chassis_event_flags & chassis_faults) != 0U)
    {
        return HBALL_Q456_OUTCOME_CONTROL_FAULT;
    }
    if ((uint32_t)(now_ms - runtime->running_since_ms) >= deadline_ms)
    {
        return HBALL_Q456_OUTCOME_DEADLINE;
    }
    if (runtime->route_phase == HBALL_Q456_ROUTE_WAIT_CONTROL_ACTIVE)
    {
        runtime->route_phase = HBALL_Q456_ROUTE_WAIT_LEFT_A;
    }
    else if (runtime->route_phase == HBALL_Q456_ROUTE_WAIT_LEFT_A)
    {
        if ((chassis_event_flags & HBALL_MISSION_CHASSIS_EVENT_LEFT_A)
            != 0U)
        {
            runtime->route_phase = HBALL_Q456_ROUTE_WAIT_REACQUIRE_A;
        }
    }
    else if (runtime->route_phase
             == HBALL_Q456_ROUTE_WAIT_REACQUIRE_A)
    {
        if ((chassis_event_flags
                & HBALL_MISSION_CHASSIS_EVENT_REACQUIRED_A)
            != 0U)
        {
            runtime->route_phase = HBALL_Q456_ROUTE_WAIT_STOPPED;
        }
    }
    else if ((runtime->route_phase == HBALL_Q456_ROUTE_WAIT_STOPPED)
             && !control_active
             && ((chassis_event_flags
                     & HBALL_MISSION_CHASSIS_EVENT_STOPPED)
                 != 0U))
    {
        return HBALL_Q456_OUTCOME_COMPLETED;
    }
    return HBALL_Q456_OUTCOME_NONE;
}

hball_q456_outcome_t hball_m33_q456_step(
    hball_m33_q456_t *runtime,
    uint32_t now_ms,
    uint16_t chassis_epoch,
    uint8_t chassis_event_flags,
    bool control_fault
)
{
    if ((runtime == NULL) || !runtime->context_valid || !runtime->running)
    {
        return HBALL_Q456_OUTCOME_NONE;
    }
    if (control_fault)
    {
        return HBALL_Q456_OUTCOME_CONTROL_FAULT;
    }
    if (chassis_epoch != runtime->epoch)
    {
        chassis_event_flags = 0U;
    }
    if (runtime->mission_id == HBALL_MISSION_Q4_A_TO_B)
    {
        return hball_m33_q4_step(runtime, now_ms, chassis_event_flags);
    }
    return hball_m33_q5_q6_step(
        runtime, now_ms, chassis_event_flags
    );
}

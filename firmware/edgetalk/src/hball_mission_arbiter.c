#include "hball_mission_arbiter.h"

#include <stddef.h>
#include <string.h>

void hball_mission_arbiter_init(hball_mission_arbiter_t *arbiter)
{
    if (arbiter == NULL)
    {
        return;
    }
    memset(arbiter, 0, sizeof(*arbiter));
    arbiter->global_state = HBALL_MISSION_STATE_SELF_TEST;
    arbiter->reason = HBALL_MISSION_REASON_NOT_READY;
}

uint16_t hball_mission_required_ready_mask(uint8_t mission_id)
{
    uint16_t required =
        HBALL_MISSION_READY_M33_ALIVE
        | HBALL_MISSION_READY_MSP_LINK
        | HBALL_MISSION_READY_CHASSIS
        | HBALL_MISSION_READY_PI_USB
        | HBALL_MISSION_READY_VISION
        | HBALL_MISSION_READY_RS00_LINK;

    if (!hball_mission_id_valid(mission_id))
    {
        return 0U;
    }
    if (mission_id != HBALL_MISSION_Q3_BALL_SEQUENCE)
    {
        required |= HBALL_MISSION_READY_IMU;
    }
    return required;
}

static bool hball_mission_epoch_is_older(uint16_t incoming, uint16_t current)
{
    const uint16_t delta = (uint16_t)(incoming - current);

    return (incoming != current) && (delta >= UINT16_C(0x8000));
}

static void hball_mission_prepare(
    hball_mission_arbiter_t *arbiter,
    const hball_mission_intent_t *intent,
    uint32_t now_ms
)
{
    const bool context_changed = !arbiter->context_valid
        || (intent->epoch != arbiter->epoch)
        || (intent->mission_id != arbiter->mission_id);

    arbiter->epoch = intent->epoch;
    arbiter->mission_id = intent->mission_id;
    arbiter->last_intent_time_ms = now_ms;
    if (context_changed)
    {
        arbiter->global_state = HBALL_MISSION_STATE_PREPARING;
        arbiter->required_mask = hball_mission_required_ready_mask(
            intent->mission_id
        );
        arbiter->reason = HBALL_MISSION_REASON_NOT_READY;
        arbiter->prepare_time_ms = now_ms;
        arbiter->ready_candidate_valid = false;
        arbiter->context_valid = true;
        arbiter->prepare_accept_total++;
    }
}

bool hball_mission_arbiter_accept_intent(
    hball_mission_arbiter_t *arbiter,
    const hball_mission_intent_t *intent,
    uint32_t now_ms
)
{
    if ((arbiter == NULL) || (intent == NULL)
        || (intent->epoch == 0U)
        || !hball_mission_id_valid(intent->mission_id))
    {
        return false;
    }
    if ((intent->command != HBALL_MISSION_COMMAND_RESET)
        && arbiter->context_valid
        && hball_mission_epoch_is_older(intent->epoch, arbiter->epoch))
    {
        arbiter->epoch_reject_total++;
        return false;
    }

    if (intent->command == HBALL_MISSION_COMMAND_PREPARE)
    {
        if ((arbiter->global_state >= HBALL_MISSION_STATE_START_PENDING)
            && (arbiter->global_state <= HBALL_MISSION_STATE_FINISHING))
        {
            if ((intent->epoch != arbiter->epoch)
                || (intent->mission_id != arbiter->mission_id))
            {
                arbiter->epoch_reject_total++;
                return false;
            }
            arbiter->last_intent_time_ms = now_ms;
            return true;
        }
        hball_mission_prepare(arbiter, intent, now_ms);
        return true;
    }

    if ((intent->command == HBALL_MISSION_COMMAND_START)
        && !arbiter->context_valid)
    {
        /*
         * M33-only reboot recovery: TI legitimately keeps retransmitting
         * START. Rebuild the matching context, but do not bypass PREPARING
         * or the 500 ms READY qualification. A later START performs launch.
         */
        hball_mission_prepare(arbiter, intent, now_ms);
        return true;
    }

    if (intent->command == HBALL_MISSION_COMMAND_RESET)
    {
        if (arbiter->context_valid
            && (arbiter->global_state >= HBALL_MISSION_STATE_START_PENDING)
            && (arbiter->global_state <= HBALL_MISSION_STATE_FINISHING))
        {
            /*
             * RESET is the cross-MCU recovery command. Keep the old context
             * for one status cycle so every consumer observes the abort;
             * a repeated RESET can then establish the caller's context.
             */
            arbiter->global_state = HBALL_MISSION_STATE_CONTROLLED_ABORT;
            arbiter->reason = HBALL_MISSION_REASON_LOCAL_FAULT;
            arbiter->ready_candidate_valid = false;
            return true;
        }
        if (!arbiter->context_valid
            || (intent->epoch != arbiter->epoch)
            || (intent->mission_id != arbiter->mission_id))
        {
            hball_mission_prepare(arbiter, intent, now_ms);
            return true;
        }
    }

    if (!arbiter->context_valid
        || (intent->epoch != arbiter->epoch)
        || (intent->mission_id != arbiter->mission_id))
    {
        arbiter->epoch_reject_total++;
        return false;
    }
    arbiter->last_intent_time_ms = now_ms;

    if (intent->command == HBALL_MISSION_COMMAND_START)
    {
        if ((arbiter->global_state >= HBALL_MISSION_STATE_START_PENDING)
            && (arbiter->global_state <= HBALL_MISSION_STATE_FINISHING))
        {
            return true;
        }
        if (arbiter->global_state != HBALL_MISSION_STATE_READY)
        {
            arbiter->reason = HBALL_MISSION_REASON_NOT_READY;
            arbiter->start_reject_total++;
            return false;
        }
        arbiter->global_state = HBALL_MISSION_STATE_START_PENDING;
        arbiter->reason = HBALL_MISSION_REASON_NONE;
        arbiter->start_event_time_ms = intent->event_time_ms;
        arbiter->start_accept_time_ms = now_ms;
        arbiter->start_accept_total++;
        return true;
    }
    if (intent->command == HBALL_MISSION_COMMAND_ABORT)
    {
        arbiter->global_state = HBALL_MISSION_STATE_CONTROLLED_ABORT;
        arbiter->reason = HBALL_MISSION_REASON_LOCAL_FAULT;
        return true;
    }
    if (intent->command == HBALL_MISSION_COMMAND_RESET)
    {
        if ((arbiter->global_state == HBALL_MISSION_STATE_PREPARING)
            || (arbiter->global_state == HBALL_MISSION_STATE_READY))
        {
            return true;
        }
        arbiter->global_state = HBALL_MISSION_STATE_PREPARING;
        arbiter->reason = HBALL_MISSION_REASON_NOT_READY;
        arbiter->ready_candidate_valid = false;
        return true;
    }
    return false;
}

void hball_mission_arbiter_update_ready(
    hball_mission_arbiter_t *arbiter,
    uint16_t ready_mask,
    uint32_t now_ms
)
{
    bool complete;

    if (arbiter == NULL)
    {
        return;
    }
    arbiter->ready_mask = ready_mask;
    if (!arbiter->context_valid)
    {
        return;
    }
    complete = (ready_mask & arbiter->required_mask)
        == arbiter->required_mask;
    if (arbiter->global_state == HBALL_MISSION_STATE_READY)
    {
        if (!complete)
        {
            arbiter->global_state = HBALL_MISSION_STATE_PREPARING;
            arbiter->reason = HBALL_MISSION_REASON_NOT_READY;
            arbiter->ready_candidate_valid = false;
        }
        return;
    }
    if (arbiter->global_state != HBALL_MISSION_STATE_PREPARING)
    {
        return;
    }
    if (!complete)
    {
        arbiter->reason = HBALL_MISSION_REASON_NOT_READY;
        arbiter->ready_candidate_valid = false;
        return;
    }
    if (!arbiter->ready_candidate_valid)
    {
        arbiter->ready_candidate_time_ms = now_ms;
        arbiter->ready_candidate_valid = true;
        return;
    }
    if ((uint32_t)(now_ms - arbiter->ready_candidate_time_ms)
        >= HBALL_MISSION_READY_STABLE_MS)
    {
        arbiter->global_state = HBALL_MISSION_STATE_READY;
        arbiter->reason = HBALL_MISSION_REASON_NONE;
    }
}

bool hball_mission_arbiter_mark_running(
    hball_mission_arbiter_t *arbiter
)
{
    if ((arbiter == NULL)
        || (arbiter->global_state != HBALL_MISSION_STATE_START_PENDING))
    {
        return false;
    }
    arbiter->global_state = HBALL_MISSION_STATE_RUNNING;
    arbiter->reason = HBALL_MISSION_REASON_NONE;
    return true;
}

bool hball_mission_arbiter_mark_completed(
    hball_mission_arbiter_t *arbiter
)
{
    if ((arbiter == NULL)
        || ((arbiter->global_state != HBALL_MISSION_STATE_RUNNING)
            && (arbiter->global_state
                != HBALL_MISSION_STATE_START_PENDING)))
    {
        return false;
    }
    arbiter->global_state = HBALL_MISSION_STATE_COMPLETED;
    arbiter->reason = HBALL_MISSION_REASON_NONE;
    return true;
}

bool hball_mission_arbiter_mark_aborted(
    hball_mission_arbiter_t *arbiter, uint8_t reason
)
{
    if ((arbiter == NULL)
        || (arbiter->global_state < HBALL_MISSION_STATE_START_PENDING)
        || (arbiter->global_state > HBALL_MISSION_STATE_FINISHING))
    {
        return false;
    }
    arbiter->global_state = HBALL_MISSION_STATE_CONTROLLED_ABORT;
    arbiter->reason = reason;
    return true;
}

bool hball_mission_arbiter_make_status(
    hball_mission_arbiter_t *arbiter,
    hball_mission_status_t *status
)
{
    if ((arbiter == NULL) || (status == NULL) || !arbiter->context_valid)
    {
        return false;
    }
    status->epoch = arbiter->epoch;
    status->mission_id = arbiter->mission_id;
    status->global_state = arbiter->global_state;
    status->ready_mask = arbiter->ready_mask;
    status->reason = arbiter->reason;
    status->status_sequence = arbiter->status_sequence++;
    return true;
}

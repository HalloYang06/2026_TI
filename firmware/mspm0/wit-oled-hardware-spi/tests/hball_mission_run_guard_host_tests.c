#include "hball_mission_run_guard.h"

#include <assert.h>
#include <string.h>

static hball_mission_client_t make_snapshot(
    uint8_t mission_id,
    uint16_t epoch,
    uint8_t global_state,
    uint32_t status_time_ms
)
{
    hball_mission_client_t snapshot;

    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.candidate_epoch = epoch;
    snapshot.selected_mission = mission_id;
    snapshot.latest_status.epoch = epoch;
    snapshot.latest_status.mission_id = mission_id;
    snapshot.latest_status.global_state = global_state;
    snapshot.last_status_ms = status_time_ms;
    snapshot.status_valid = true;
    return snapshot;
}

static hball_mission_run_decision_t evaluate_remote(
    uint8_t global_state,
    uint32_t status_time_ms,
    uint32_t now_ms
)
{
    hball_mission_policy_t policy;
    hball_mission_client_t snapshot = make_snapshot(
        HBALL_MISSION_Q4_A_TO_B,
        7U,
        global_state,
        status_time_ms
    );

    assert(hball_mission_policy_get(HBALL_MISSION_Q4_A_TO_B, &policy));
    return hball_mission_run_guard_evaluate(
        &policy,
        HBALL_MISSION_Q4_A_TO_B,
        7U,
        &snapshot,
        now_ms
    );
}

static void test_q2_local_run_does_not_depend_on_m33_status(void)
{
    hball_mission_policy_t policy;

    assert(hball_mission_policy_get(HBALL_MISSION_Q2_FAST_LAP, &policy));
    assert(hball_mission_run_guard_evaluate(
        &policy,
        HBALL_MISSION_Q2_FAST_LAP,
        3U,
        NULL,
        5000U
    ) == HBALL_MISSION_RUN_CONTINUE);
}

static void test_remote_run_requires_matching_fresh_running_status(void)
{
    hball_mission_policy_t policy;
    hball_mission_client_t snapshot = make_snapshot(
        HBALL_MISSION_Q4_A_TO_B,
        7U,
        HBALL_MISSION_STATE_RUNNING,
        1000U
    );

    assert(hball_mission_policy_get(HBALL_MISSION_Q4_A_TO_B, &policy));
    assert(hball_mission_run_guard_evaluate(
        &policy,
        HBALL_MISSION_Q4_A_TO_B,
        7U,
        &snapshot,
        1150U
    ) == HBALL_MISSION_RUN_CONTINUE);

    snapshot.selected_mission = HBALL_MISSION_Q5_CENTER_LAP;
    assert(hball_mission_run_guard_evaluate(
        &policy,
        HBALL_MISSION_Q4_A_TO_B,
        7U,
        &snapshot,
        1150U
    ) == HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE);

    snapshot.selected_mission = HBALL_MISSION_Q4_A_TO_B;
    snapshot.latest_status.epoch = 8U;
    assert(hball_mission_run_guard_evaluate(
        &policy,
        HBALL_MISSION_Q4_A_TO_B,
        7U,
        &snapshot,
        1150U
    ) == HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE);

    snapshot.latest_status.epoch = 7U;
    assert(hball_mission_run_guard_evaluate(
        &policy,
        HBALL_MISSION_Q4_A_TO_B,
        7U,
        &snapshot,
        1151U
    ) == HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE);
}

static void test_remote_terminal_states_request_a_local_stop(void)
{
    assert(evaluate_remote(
        HBALL_MISSION_STATE_FINISHING, 1000U, 1001U
    ) == HBALL_MISSION_RUN_STOP_REMOTE_COMPLETE);
    assert(evaluate_remote(
        HBALL_MISSION_STATE_COMPLETED, 1000U, 1001U
    ) == HBALL_MISSION_RUN_STOP_REMOTE_COMPLETE);
    assert(evaluate_remote(
        HBALL_MISSION_STATE_CONTROLLED_ABORT, 1000U, 1001U
    ) == HBALL_MISSION_RUN_STOP_REMOTE_ABORT);
    assert(evaluate_remote(
        HBALL_MISSION_STATE_FAULT_LATCHED, 1000U, 1001U
    ) == HBALL_MISSION_RUN_STOP_REMOTE_ABORT);
    assert(evaluate_remote(
        HBALL_MISSION_STATE_PREPARING, 1000U, 1001U
    ) == HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE);
}

static void test_invalid_or_chassis_inhibited_policy_fails_closed(void)
{
    hball_mission_policy_t policy;
    hball_mission_client_t snapshot = make_snapshot(
        HBALL_MISSION_Q3_BALL_SEQUENCE,
        4U,
        HBALL_MISSION_STATE_RUNNING,
        1000U
    );

    assert(hball_mission_run_guard_evaluate(
        NULL,
        HBALL_MISSION_Q4_A_TO_B,
        7U,
        &snapshot,
        1001U
    ) == HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE);

    assert(hball_mission_policy_get(HBALL_MISSION_Q3_BALL_SEQUENCE, &policy));
    assert(hball_mission_run_guard_evaluate(
        &policy,
        HBALL_MISSION_Q3_BALL_SEQUENCE,
        4U,
        &snapshot,
        1001U
    ) == HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE);
}

int main(void)
{
    test_q2_local_run_does_not_depend_on_m33_status();
    test_remote_run_requires_matching_fresh_running_status();
    test_remote_terminal_states_request_a_local_stop();
    test_invalid_or_chassis_inhibited_policy_fails_closed();
    return 0;
}

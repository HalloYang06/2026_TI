from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
MAIN = PROJECT / "main.c"


def _lap_test_source() -> str:
    source = MAIN.read_text(encoding="utf-8")
    return source[
        source.index("static void lap_test_once(void)\n{") :
        source.index("void TIMER_0_INST_IRQHandler(void)")
    ]


def test_initial_line_loss_requests_abort_before_chassis_finish() -> None:
    lap = _lap_test_source()
    branch_start = lap.index("if (line_mask == 0U)")
    branch_end = lap.index("competition_runtime_wait_ms(500U);", branch_start)
    branch = lap[branch_start:branch_end]

    assert branch.index("hball_can_mission_request_abort(tick_ms)") < branch.index(
        "hball_can_mission_chassis_finish"
    )
    assert "HBALL_MISSION_CHASSIS_EVENT_STOPPED" in branch
    assert "HBALL_MISSION_CHASSIS_EVENT_LINE_LOST" in branch
    assert "HBALL_MISSION_CHASSIS_EVENT_LOCAL_FAULT" in branch


def test_runtime_line_loss_finishes_with_explicit_fault_facts() -> None:
    lap = _lap_test_source()
    branch_start = lap.index("if (follower_output.lost_timeout)")
    branch_end = lap.index("if (wheel_control_due", branch_start)
    branch = lap[branch_start:branch_end]

    assert "finish_event_flags = HBALL_MISSION_CHASSIS_EVENT_STOPPED" in branch
    assert "| HBALL_MISSION_CHASSIS_EVENT_LINE_LOST" in branch
    assert "| HBALL_MISSION_CHASSIS_EVENT_LOCAL_FAULT" in branch

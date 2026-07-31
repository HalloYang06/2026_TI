from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]


def test_active_lap_checks_remote_run_permission_before_and_during_motion() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    lap = main[
        main.index("static void lap_test_once(void)\n{") :
        main.index("void TIMER_0_INST_IRQHandler(void)")
    ]

    assert '#include "hball_mission_run_guard.h"' in main
    assert "mission_id = selected_task;" in lap
    assert "mission_epoch = mission_snapshot.candidate_epoch;" in lap
    assert lap.count("hball_mission_run_guard_evaluate(") >= 2

    prestart_guard = lap.index("hball_mission_run_guard_evaluate(")
    motor_start = lap.index("chassis_actuator_start_synchronized(")
    loop_start = lap.index("while (1)", motor_start)
    runtime_guard = lap.index("hball_mission_run_guard_evaluate(", loop_start)
    follower_step = lap.index("line_follower_step(", loop_start)

    assert prestart_guard < motor_start
    assert loop_start < runtime_guard < follower_step


def test_remote_stop_does_not_report_a_success_marker() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    lap = main[
        main.index("static void lap_test_once(void)\n{") :
        main.index("void TIMER_0_INST_IRQHandler(void)")
    ]
    compact = "".join(lap.split())

    assert "HBALL_MISSION_RUN_STOP_REMOTE_COMPLETE" in lap
    assert "HBALL_MISSION_RUN_STOP_REMOTE_ABORT" in lap
    assert "HBALL_MISSION_RUN_STOP_REMOTE_UNAVAILABLE" in lap
    assert "finish_event_flags=HBALL_MISSION_CHASSIS_EVENT_STOPPED;" in compact
    assert "hball_can_mission_chassis_finish(finish_event_flags,tick_ms);" in compact

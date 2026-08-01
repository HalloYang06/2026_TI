from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]


def test_msp_can_port_carries_only_mission_control_frames() -> None:
    source = (PROJECT / "Drivers" / "CAN" / "hball_can_port.c").read_text(
        encoding="utf-8"
    )

    assert "hball_mission_encode_intent" in source
    assert "hball_mission_encode_chassis_status" in source
    assert "hball_mission_decode_status" in source
    assert "hball_mission_decode_ui" in source
    assert "hball_can_mission_request_start" in source
    assert "HBALL_CAN_MOTOR_COMMAND_TX_ENABLED 0U" in source
    assert "element->data[index] = frame->data[index]" in source
    assert "target->data[index] = (uint8_t)source->data[index]" in source
    assert "memcpy(element->data" not in source


def test_chassis_route_events_are_latched_only_from_observed_facts() -> None:
    header = (PROJECT / "Drivers" / "CAN" / "hball_can_port.h").read_text(
        encoding="utf-8"
    )
    source = (PROJECT / "Drivers" / "CAN" / "hball_can_port.c").read_text(
        encoding="utf-8"
    )
    start = source[
        source.index("void hball_can_mission_chassis_start") :
        source.index("void hball_can_mission_chassis_latch_events")
    ]
    latch = source[
        source.index("void hball_can_mission_chassis_latch_events") :
        source.index("void hball_can_mission_chassis_finish")
    ]
    finish = source[
        source.index("void hball_can_mission_chassis_finish") :
        source.index("static void hball_can_update_wit_freshness")
    ]

    assert "hball_can_mission_chassis_latch_events(uint8_t event_flags)" in header
    assert "HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE" in start
    assert "HBALL_MISSION_CHASSIS_EVENT_LEFT_A" not in start
    assert "g_hball_chassis_events | event_flags" in latch
    assert "g_hball_chassis_phase" not in latch
    assert "g_hball_chassis_events | event_flags" in finish
    assert "HBALL_MISSION_CHASSIS_EVENT_STOPPED" in finish
    assert (
        "(uint8_t)~HBALL_MISSION_CHASSIS_EVENT_CONTROL_ACTIVE" in finish
    )


def test_mission_intent_slots_cannot_be_starved_by_imu_telemetry() -> None:
    source = (PROJECT / "Drivers" / "CAN" / "hball_can_port.c").read_text(
        encoding="utf-8"
    )
    service = source[
        source.index("void hball_can_port_tick_1ms") :
        source.index("static void hball_can_record_rx")
    ]

    assert "intent_due = ((now_ms % 50U) == 7U)" in service
    assert "? !telemetry_due" not in service


def test_target_startup_does_not_request_semihosted_argv() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")

    assert "__ARM_use_no_argv" in main


def test_keil_project_builds_shared_mission_protocol_and_msp_client() -> None:
    project = (
        PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx"
    ).read_text(encoding="utf-8")

    assert "hball_mission_can.c" in project
    assert "hball_mission_client.c" in project
    assert "shared\\protocol" in project

    build = (PROJECT / "tools" / "build-keil.ps1").read_text(encoding="utf-8")
    assert "Drivers\\CAN\\hball_mission_client.c" in build
    assert "shared\\protocol" in build
    assert "hball_mission_can.c" in build

    generator = (
        PROJECT / "tools" / "generate-keil-project.ps1"
    ).read_text(encoding="utf-8")
    assert "hball_mission_client.c" in generator
    assert "hball_mission_can.c" in generator
    assert "shared\\protocol" in generator

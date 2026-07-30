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

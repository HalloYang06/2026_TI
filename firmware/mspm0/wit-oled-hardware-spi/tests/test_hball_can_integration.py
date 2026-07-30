from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
CAN_DIR = PROJECT / "Drivers" / "CAN"


def test_mspm0_sysconfig_is_1m_classic_can_on_verified_pins() -> None:
    syscfg = (PROJECT / "wit-oled-hardware-spi.syscfg").read_text(encoding="utf-8")

    assert 'MCAN1.$name' in syscfg
    assert "MCAN1.fdMode" in syscfg and "false" in syscfg
    assert "MCAN1.brsEnable" in syscfg and "false" in syscfg
    assert "MCAN1.desiredNomRate" in syscfg and "1000" in syscfg
    assert 'MCAN1.peripheral.txPin.$assign = "PA26"' in syscfg
    assert 'MCAN1.peripheral.rxPin.$assign = "PA27"' in syscfg
    assert 'mux2.inputSelect = "CANCLKMUX_PLLCLK1_OUT"' in syscfg


def test_mspm0_can_port_is_integrated_without_motor_commands() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    interrupt = (PROJECT / "Drivers" / "MSPM0" / "interrupt.c").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(encoding="utf-8")
    port = (CAN_DIR / "hball_can_port.c").read_text(encoding="utf-8")

    assert "hball_can_port_init();" in main
    assert "hball_can_port_tick_1ms(tick_ms);" in interrupt
    assert "Drivers\\CAN\\hball_can_protocol.c" in build
    assert "Drivers\\CAN\\hball_can_port.c" in build
    assert "DL_MCAN_writeMsgRam" in port
    assert "HBALL_CAN_MOTOR_COMMAND_TX_ENABLED 0U" in port
    for forbidden in ("motor_enable", "set_zero", "loc_ref", "limit_cur"):
        assert forbidden not in port.lower()

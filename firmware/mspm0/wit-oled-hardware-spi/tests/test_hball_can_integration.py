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


def test_wit_uart_source_and_generated_config_are_115200_baud() -> None:
    syscfg = (PROJECT / "wit-oled-hardware-spi.syscfg").read_text(encoding="utf-8")
    generated_c = (PROJECT / "Debug" / "ti_msp_dl_config.c").read_text(
        encoding="utf-8"
    )
    generated_h = (PROJECT / "Debug" / "ti_msp_dl_config.h").read_text(
        encoding="utf-8"
    )

    assert "UART3.targetBaudRate                   = 115200;" in syscfg
    assert "Target baud rate: 115200" in generated_c
    assert "UART_WIT_BAUD_RATE                                              (115200)" in generated_h
    assert "UART_WIT_IBRD_40_MHZ_115200_BAUD" in generated_c
    assert "UART_WIT_FBRD_40_MHZ_115200_BAUD" in generated_c


def test_wit_dma_starts_before_non_returning_application_mode() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")

    assert main.count("WIT_Init();") == 1
    assert main.index("WIT_Init();") < main.index("#if APP_MODE == APP_MODE_LCD_TEST")


def test_wit_dma_completion_and_uart_timeout_share_stream_parser() -> None:
    interrupt = (PROJECT / "Drivers" / "MSPM0" / "interrupt.c").read_text(
        encoding="utf-8"
    )
    wit = (PROJECT / "Drivers" / "WIT" / "wit.c").read_text(encoding="utf-8")
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(encoding="utf-8")

    assert "void DMA_IRQHandler(void)" in interrupt
    assert "WIT_ProcessBytes" in interrupt
    assert "DL_DMA_EVENT_IIDX_DMACH0" in interrupt
    assert "DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL0)" in wit
    assert "Drivers\\WIT\\wit_parser.c" in build


def test_jy901s_accel_gyro_angle_reports_are_enabled_before_dma() -> None:
    wit = (PROJECT / "Drivers" / "WIT" / "wit.c").read_text(encoding="utf-8")
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(encoding="utf-8")

    configure = wit.index("wit_jy901s_enable_control_reports();")
    arm_dma = wit.index("DL_DMA_setSrcAddr(")
    assert configure < arm_dma
    assert "Drivers\\WIT\\wit_jy901s_config.c" in build


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
    assert "Drivers\\CAN\\hball_can_recovery.c" in build
    assert "Drivers\\CAN\\hball_can_port.c" in build
    assert "DL_MCAN_writeMsgRam" in port
    assert "DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_NORMAL)" in port
    assert "bus_off_recovery_attempts" in port
    assert "HBALL_CAN_MOTOR_COMMAND_TX_ENABLED 0U" in port
    for forbidden in ("motor_enable", "set_zero", "loc_ref", "limit_cur"):
        assert forbidden not in port.lower()


def test_can_fifo_drain_rejects_systick_irq_reentry() -> None:
    port = (CAN_DIR / "hball_can_port.c").read_text(encoding="utf-8")
    drain = port.split("static void hball_can_drain_fifo0(void)\n{", 1)[1]
    drain = drain.split("\nvoid MCAN0_INST_IRQHandler", 1)[0]

    assert "static volatile bool g_hball_rx_drain_active;" in port
    assert "if (g_hball_rx_drain_active)" in drain
    assert drain.index("g_hball_rx_drain_active = true;") < drain.index(
        "DL_MCAN_getRxFIFOStatus"
    )
    assert drain.rindex("g_hball_rx_drain_active = false;") > drain.rindex(
        "DL_MCAN_getRxFIFOStatus"
    )

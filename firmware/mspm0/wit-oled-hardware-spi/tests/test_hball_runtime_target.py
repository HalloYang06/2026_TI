from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
RUNTIME_DIR = PROJECT / "App" / "Runtime"


def test_target_adapter_binds_dispatcher_to_bounded_can_and_imu_services() -> None:
    header = (RUNTIME_DIR / "hball_runtime_target.h").read_text(
        encoding="utf-8"
    )
    source = (RUNTIME_DIR / "hball_runtime_target.c").read_text(
        encoding="utf-8"
    )

    assert "hball_runtime_dispatcher_init" in source
    assert "hball_runtime_dispatcher_tick_isr" in source
    assert "hball_runtime_dispatcher_poll" in source
    assert source.count("hball_can_port_tick_1ms(now_ms);") == 1
    assert "hball_runtime_services_can_enabled()" in source
    assert "hball_runtime_services_imu_enabled()" in source
    assert "WIT_Service(WIT_FOREGROUND_BUDGET_PER_SERVICE);" in source
    assert "__get_PRIMASK()" in source
    assert "__disable_irq()" in source
    assert "__enable_irq()" in source
    assert "extern volatile hball_runtime_target_stats_t" in header
    assert "can_deadline_miss_total" in header
    assert "can_service_total" in header
    assert "imu_deadline_miss_total" in header
    assert "imu_service_total" in header
    assert "imu_pending" in header


def test_keil_build_includes_runtime_target_adapter() -> None:
    project = (PROJECT / "Keil" / "wit-oled-hardware-spi.uvprojx").read_text(
        encoding="utf-8"
    )
    build = (PROJECT / "tools" / "build-keil.ps1").read_text(
        encoding="utf-8"
    )
    generator = (PROJECT / "tools" / "generate-keil-project.ps1").read_text(
        encoding="utf-8"
    )

    assert "hball_runtime_target.c" in project
    assert "App\\Runtime\\hball_runtime_target.c" in build
    assert "hball_runtime_target.c" in generator


def test_systick_only_publishes_runtime_tick() -> None:
    interrupt = (
        PROJECT / "Drivers" / "MSPM0" / "interrupt.c"
    ).read_text(encoding="utf-8")
    systick = interrupt[
        interrupt.index("void SysTick_Handler(void)") :
        interrupt.index("#if defined UART_BNO08X_INST_IRQHandler")
    ]

    assert "hball_can_port_tick_1ms" not in systick
    assert (
        "void SysTick_Handler(void)\n"
        "{\n"
        "    tick_ms++;\n"
        "    hball_runtime_target_tick_isr();\n"
        "}\n\n"
    ) == systick


def test_runtime_target_starts_before_systick_and_is_polled_in_foreground() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    init = main[
        main.index("int main(void){") :
        main.index("#if APP_MODE == APP_MODE_LCD_TEST")
    ]
    menu = main[
        main.index("static uint8_t select_car_task(void)\n{") :
        main.index("static void render_mission_menu(",
                   main.index("static uint8_t select_car_task(void)\n{"))
    ]
    runtime_wait = main[
        main.index("static void competition_runtime_wait_ms(uint32_t duration_ms)\n{") :
        main.index("static uint8_t select_car_task(void)\n{",
                   main.index("static void competition_runtime_wait_ms"))
    ]

    assert init.index("hball_can_port_init();") < init.index(
        "hball_runtime_target_init();"
    ) < init.index("SysTick_Init();")
    assert "hball_runtime_target_poll(tick_ms);" in menu
    assert menu.count("competition_runtime_wait_ms(5U);") == 2
    assert menu.count("competition_runtime_wait_ms(50U);") == 1
    assert "delay_cycles(CPUCLK_FREQ / 200U);" not in menu
    assert "hball_runtime_target_poll(tick_ms);" in runtime_wait
    assert "__WFI();" in runtime_wait


def test_competition_follower_and_q56_nonblocking_stop_service_runtime() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    lap = main[
        main.index("static void lap_test_once(void)\n{") :
        main.index("void TIMER_0_INST_IRQHandler(void)")
    ]

    assert lap.count("competition_runtime_wait_ms(10U);") == 1
    assert "competition_runtime_wait_ms(20U);" not in lap
    assert "delay_cycles(CPUCLK_FREQ / 100U);" not in lap
    assert "delay_cycles(CPUCLK_FREQ / 50U);" not in lap
    assert "run_key_event = get_task_key_event();" in lap
    assert "hball_can_mission_request_abort(tick_ms)" in lap
    assert "SW1 CANCEL" in lap

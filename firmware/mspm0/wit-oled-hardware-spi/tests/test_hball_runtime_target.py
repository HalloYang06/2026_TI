from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
RUNTIME_DIR = PROJECT / "App" / "Runtime"


def test_target_adapter_binds_dispatcher_to_bounded_can_service() -> None:
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
    assert "__get_PRIMASK()" in source
    assert "__disable_irq()" in source
    assert "__enable_irq()" in source
    assert "extern volatile hball_runtime_target_stats_t" in header
    assert "can_deadline_miss_total" in header
    assert "can_service_total" in header


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

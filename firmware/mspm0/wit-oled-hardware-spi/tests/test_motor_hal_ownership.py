from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
MOTOR_DIR = PROJECT / "Drivers" / "MOTOR"


def test_only_motor_hal_writes_stby_direction_and_pwm_registers() -> None:
    motor_source = (MOTOR_DIR / "motor.c").read_text(encoding="utf-8")
    motor_header = (MOTOR_DIR / "motor.h").read_text(encoding="utf-8")

    assert "void motor_driver_enable(void);" in motor_header
    assert "void motor_driver_disable(void);" in motor_header
    for private_token in (
        "motor_gpio_STBY_PIN",
        "motor_gpio_AIN1_PIN",
        "motor_gpio_AIN2_PIN",
        "motor_gpio_BIN1_PIN",
        "motor_gpio_BIN2_PIN",
        "DL_TimerA_setCaptureCompareValue(PWM_A_INST",
    ):
        assert private_token in motor_source
        assert private_token not in motor_header

    owned_sources = [
        PROJECT / "main.c",
        PROJECT / "Drivers" / "GRAY" / "track.c",
        PROJECT / "Drivers" / "MSPM0" / "interrupt.c",
    ]
    for source_path in owned_sources:
        source = source_path.read_text(encoding="utf-8")
        assert "motor_gpio_STBY_PIN" not in source
        assert "motor_gpio_AIN1_PIN" not in source
        assert "motor_gpio_AIN2_PIN" not in source
        assert "motor_gpio_BIN1_PIN" not in source
        assert "motor_gpio_BIN2_PIN" not in source
        assert "DL_TimerA_setCaptureCompareValue(PWM_A_INST" not in source


def test_competition_start_and_stop_use_motor_hal_standby_api() -> None:
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    motor = (MOTOR_DIR / "motor.c").read_text(encoding="utf-8")

    assert "motor_driver_disable();" in main
    assert "motor_driver_enable();" in main
    assert "motor_driver_disable();" in motor
    assert "motor_driver_enable();" in motor

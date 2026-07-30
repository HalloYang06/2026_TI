from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]


def _function_body(source: str, signature: str, next_signature: str) -> str:
    start = source.index(signature)
    end = source.index(next_signature, start)
    return source[start:end]


def test_lap_start_preloads_both_wheels_before_shared_enable() -> None:
    motor = (PROJECT / "Drivers" / "MOTOR" / "motor.c").read_text(
        encoding="utf-8"
    )
    main = (PROJECT / "main.c").read_text(encoding="utf-8")
    start_body = _function_body(
        motor,
        "void motor_start_synchronized(float pwm1,float pwm2)\n{",
        "void set_motor_speed",
    )

    disable = start_body.index(
        "DL_GPIO_clearPins(motor_gpio_PORT, motor_gpio_STBY_PIN)"
    )
    preload = start_body.index("motor_pwm_set(pwm1, pwm2)")
    enable = start_body.index(
        "DL_GPIO_setPins(motor_gpio_PORT, motor_gpio_STBY_PIN)"
    )
    assert disable < preload < enable
    assert "motor_start_synchronized((float)commanded_duty_left" in main


def test_motor_pwm_keeps_verified_per_wheel_direction_writes() -> None:
    motor = (PROJECT / "Drivers" / "MOTOR" / "motor.c").read_text(
        encoding="utf-8"
    )
    body = motor[motor.index("void motor_pwm_set(float pwm1,float pwm2)\n{") :]

    assert "left_motor_dir(1)" in body
    assert "left_motor_dir(0)" in body
    assert "right_motor_dir(1)" in body
    assert "right_motor_dir(0)" in body
    assert "DL_GPIO_writePinsVal" not in body

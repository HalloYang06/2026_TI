from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
CONTROL = PROJECT / "App" / "Control"
MOTOR = PROJECT / "Drivers" / "MOTOR"

MOTOR_CALLS = (
    "motor_init(",
    "motor_stop(",
    "motor_driver_enable(",
    "motor_driver_disable(",
    "motor_start_synchronized(",
    "motor_pwm_set(",
    "set_motor_speed(",
)


def test_chassis_actuator_is_the_only_motor_hal_client() -> None:
    adapter = (CONTROL / "chassis_actuator.c").read_text(encoding="utf-8")

    assert '#include "motor.h"' in adapter
    for source_path in PROJECT.rglob("*.[ch]"):
        if source_path == CONTROL / "chassis_actuator.c":
            continue
        if source_path.is_relative_to(MOTOR):
            continue

        source = source_path.read_text(encoding="utf-8", errors="ignore")
        assert '#include "motor.h"' not in source, source_path
        for call in MOTOR_CALLS:
            assert call not in source, f"{source_path}: {call}"


def test_chassis_actuator_preserves_existing_motor_operations() -> None:
    header = (CONTROL / "chassis_actuator.h").read_text(encoding="utf-8")
    source = (CONTROL / "chassis_actuator.c").read_text(encoding="utf-8")

    for operation in (
        "chassis_actuator_init",
        "chassis_actuator_stop",
        "chassis_actuator_enable",
        "chassis_actuator_disable",
        "chassis_actuator_start_synchronized",
        "chassis_actuator_set_wheel_speed",
        "chassis_actuator_set_pwm",
    ):
        assert operation in header
        assert operation in source

    assert "motor_start_synchronized(left_pwm, right_pwm);" in source
    assert "motor_pwm_set(left_pwm, right_pwm);" in source

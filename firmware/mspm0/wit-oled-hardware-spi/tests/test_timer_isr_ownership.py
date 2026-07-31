from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
MAIN = (PROJECT / "main.c").read_text(encoding="utf-8")


def _timer0_isr_body() -> str:
    return MAIN.split("void TIMER_0_INST_IRQHandler(void)", 1)[1].split(
        "void TIMER_1_INST_IRQHandler(void)", 1
    )[0]


def test_timer0_isr_only_publishes_encoder_sample() -> None:
    body = _timer0_isr_body()

    assert "Get_Encoder_countA" in body
    assert "Get_Encoder_countB" in body
    assert "encoderA_cnt =" in body
    assert "encoderB_cnt =" in body
    for forbidden in (
        "PID_Update",
        "motor_pwm_set",
        "set_motor_speed",
        "wit_data",
        "speed_pid_enabled",
        "LEFT.",
        "RIGHT.",
        "ANGLE.",
    ):
        assert forbidden not in body


def test_removed_timer_motor_gate_cannot_be_reenabled() -> None:
    for obsolete in (
        "SPEED_PID_DISABLED",
        "SPEED_PID_ENABLED",
        "speed_pid_enabled",
        "pwm1_out",
        "pwm2_out",
    ):
        assert obsolete not in MAIN

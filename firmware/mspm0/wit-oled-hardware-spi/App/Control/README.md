# MSPM0 chassis control boundary

`ChassisActuator` is the application-side owner of the brushed chassis motor
HAL. Mission, line-following, calibration, and test code may request an
actuator operation through this module, but must not include `motor.h` or call
the motor HAL directly.

The first migration step is deliberately behavior-preserving: the adapter
keeps the existing PWM signs, limits, startup ordering, and stop sequences.
It does not tune Q2/Q4 or move PID work into an interrupt. Follow-up slices can
replace direct requests with timestamped `MotionIntent` values after the
current hardware baselines have been reverified.

Ownership rules:

- `Drivers/MOTOR/motor.c` alone writes direction GPIO, PWM registers, and
  STBY.
- `App/Control/chassis_actuator.c` alone calls the public Motor HAL.
- Interrupts may publish encoder samples but cannot call either layer.
- Competition initialization and terminal stop paths keep their explicit
  STBY-low requests; the adapter does not silently alter legacy sequencing.
- No automated test may enable the actuator or start a mission.

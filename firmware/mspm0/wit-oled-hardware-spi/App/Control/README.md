# MSPM0 chassis control boundary

`ChassisActuator` is the application-side owner of the brushed chassis motor
HAL. Mission, line-following, calibration, and test code may request an
actuator operation through this module, but must not include `motor.h` or call
the motor HAL directly.

`LineSnapshot` is the hardware-independent decoder for the eight-channel,
active-low line sensor sample. It owns the frozen Q2 weights, active-channel
count, weighted integer error, and adjacent-channel pattern query. It accepts
only a raw byte plus timestamp and must not read GPIO, mission state, display,
transport, or actuator APIs. The GPIO sampling site remains outside this
module.

`WheelControl` owns the competition lap controller's two wheel PID instances,
100 ms encoder-delta normalization, integral bounds, measured PWM feed-forward,
and output slew. It consumes atomic encoder counts and requested wheel speeds;
it returns a PWM request without reading registers or calling the actuator.

`ChassisMotionProfile` is a pure minimum-jerk scale generator. Mission code
chooses the start time and duration, applies the returned scale to a
`MotionIntent`, and remains responsible for normal-stop versus hard-fault stop
policy. The profile has no actuator, transport, sensor, or mission dependency.

`MotionIntent` is the timestamped, hardware-independent handoff between line
following and wheel control. Its speed unit remains encoder counts per 100 ms
to preserve the verified controller. A valid intent may still represent
stateful lost-line recovery even when the current `LineSnapshot` is invalid.

`LineFollower` owns the active Q2, Q4, and stable-lap line-following state. It
runs from the 10 ms competition loop, consumes only a `LineSnapshot`, profile,
timestamp, and the caller's start-line straight-through decision, then emits a
`MotionIntent`. Q2 curve timing, lost-line search, stable-lap filtering and
reacquisition latching live here. Finish detection, mission deadlines, display,
encoder sampling, wheel PID, and actuation remain outside this module. A
reacquisition stays pending until the 100 ms wheel controller has successfully
applied the corresponding intent and the caller acknowledges it.

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
- `LineSnapshot` produces facts only; it cannot select a task or request
  chassis motion.
- `LineFollower` cannot read GPIO, mission globals, displays, transports,
  encoders, or actuator state.
- `WheelControl` alone updates the active lap runtime's wheel PID state; the
  caller owns atomic encoder sampling and the final actuator request. It may
  consume only an explicitly valid `MotionIntent`.

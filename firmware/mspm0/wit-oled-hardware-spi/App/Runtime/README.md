# MSPM0 runtime coordination

This directory contains small, hardware-independent coordination primitives for
the cooperative MSPM0 runtime. It does not own mission phases, control laws, CAN
frames, IMU parsing, or actuator writes.

`hball_runtime_services` publishes one atomic service-enable mask. The mission
entry path updates the mask, while interrupt/foreground service code only reads
it. Menu mode enables CAN and IMU work so the operator can select any task. Q2
disables their competition business work while the verified local line follower
runs; Q3-Q6 keep both services enabled.

No function in this directory may call HAL, motor, CAN-port, WIT, LCD, delay, or
blocking APIs.

`hball_coop_scheduler` centralizes the fixed 1/5/10/100 ms release periods.
SysTick calls only `tick_isr`, which increments saturating pending counters and
deadline-miss statistics. The foreground dispatcher consumes at most bounded
work per pass. Its `take` operation must be wrapped in a target critical section;
the scheduler core remains hardware- and RTOS-independent for host testing.

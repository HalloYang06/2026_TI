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

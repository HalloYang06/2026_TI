# Gray and line-sensor hardware boundary

`LineSensorPort` is the only owner of the eight `track_PIN_*` GPIO reads. It
returns either the physical raw byte (`1` means the pin is high) or the
active-line mask (`1` means the active-low sensor sees the line). Application
code must pass the raw byte to `LineSnapshot`; it must not read these GPIOs
directly.

The older `track.c` controller remains temporarily for non-competition test
modes, but it consumes `LineSensorPort` and is not the active Q2/Q3 controller.
Its motor decisions and the legacy software-I2C implementation in `gray.c`
must not be reused in new mission code. They can be removed only after their
remaining callers have been migrated and verified.

Ownership rules:

- `line_sensor_port.c` alone reads `track_PIN_0` through `track_PIN_7`.
- `LineSensorPort` reports hardware facts only; it has no task, timing, LCD,
  CAN, PID, or motor dependency.
- `LineSnapshot` owns active count, weighted error, validity, and adjacent
  pattern decoding.
- Line-following state consumes snapshots and returns intent; it cannot read
  GPIO directly.

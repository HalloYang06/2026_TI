# MSPM0 generic chassis services

This directory owns task-independent chassis execution and facts. It may
combine timestamped line, encoder, and actuator-neutral state, but it must not
own Q2-Q6 phases, deadlines, completion decisions, CAN frames, displays, or
motor writes.

`RouteMarkerDetector` converts a `LineSnapshot` stream into generic facts:
whether a configured marker pattern is present, whether the starting marker
has been continuously cleared, and whether a later marker has passed its
spatial and time confirmation gates. The EdgeTalk M33 mission runtime decides
what those transported facts mean. The detector has no task ID and cannot stop
or command the chassis.

Allowed direction:

```text
runtime/chassis executor -> App/Chassis -> App/Control facts
```

`App/Chassis` must not call `App/Mission`, transports, the display, the motor
HAL, or any blocking API.

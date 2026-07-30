# 载车钢球—水管控制架构

## 推荐数据链

```text
Raspberry Pi camera (100 Hz)
  └─ ball position + confidence + capture timestamp + frame id + CRC
       │ UART, recommended 921600 bit/s or faster
       ▼
PSOC Edge E84
  ├─ joystick / vehicle high-level state machine
  ├─ map segment and target-speed scheduling
  ├─ 200 Hz delayed KF / linear ESO + gain-scheduled LQI
  ├─ predictive pipe-edge recovery and linkage inverse kinematics
  ├─ RS00 position/speed/torque/temperature safety limits
  └─ RS00 CAN 1 Mbit/s motion-mode command and feedback

MSPM0G3507
  ├─ current WIT source: about 29.1 complete groups/s at 9600 bit/s
  └─ source-sequenced/timestamped IMU publication
       │ dedicated SPI/UART/CAN-FD
       └──────────────────────────────► PSOC Edge E84
```

The balancing and RS00 safety loops run on the PSOC Edge E84, which is directly
connected to the actuator CAN bus. The MSPM0 is an IMU acquisition
coprocessor. Linux remains only a vision source, so a Raspberry-Pi delay or
restart cannot bypass the motor limits and watchdog on the PSOC.

## Controller

The augmented observer state is:

```text
z = [ball_position, ball_velocity, lumped_acceleration_disturbance]
```

The delayed camera position is not treated as a current-time measurement:

```text
x(t-delay) ~= x(t) - delay*v(t)
              + 0.5*delay^2*(b*u_effective + disturbance)
```

The command is:

```text
pipe_angle_command =
    vehicle_acceleration/g - vehicle_pitch       IMU/planned feedforward
  - estimated_disturbance/b                      low-bandwidth linear ESO
  - Kx*position_error - Kv*velocity - Ki*integral
```

It then passes through:

1. predictive edge recovery;
2. ±6 degree pipe-angle limit;
3. 0.35 rad/s pipe command-rate limit;
4. linkage inverse kinematics;
5. RS00 position/speed/torque/temperature limits.

The disturbance observer should remain slower than the delayed camera path.
A high-bandwidth standalone LADRC will interpret delayed position as a new
disturbance and can produce a limit cycle on the slippery pipe.

## Real-time task budget

The current WIT UART is 9600 bit/s. Three 11-byte 8N1 frames require at least
330 bits, so there can be at most about 29.1 complete accel/gyro/attitude
groups per second. The MSPM0 timestamps and source-sequences those groups.
A 200 Hz CAN mirror is publication scheduling, not a 5 ms IMU source period.

| Device/task | Suggested method |
|---|---|
| MSPM0, source event ≈29 Hz | assemble WIT frames and timestamp the unique group |
| MSPM0, CAN scheduler | publish source sequence/age; repeated mirrors do not refresh freshness |
| PSOC, 200 Hz | consume the latest timestamped IMU sample with zero-order hold |
| PSOC, 200 Hz | 3-state observer, LQI, feedforward and edge supervisor |
| PSOC, 500 Hz | linkage conversion, command interpolation and safety limits |
| PSOC, 500 Hz | pre-built RS00 CAN frame and transmit-deadline watchdog |

Run the position observer/LQI at 200 Hz and hold the latest unique IMU source
sample between arrivals. Motor feedback and the RS00 safety layer can still
run at 500 Hz. A new camera or IMU sample is used only once according to its
source sequence and capture timestamp.

The IMU is intentionally mounted on the chassis: it measures base pitch and
base acceleration for feedforward. It does not measure pipe angle relative to
the chassis. That angle comes from the RS00 encoder and four-bar forward
kinematics:

```text
absolute_pipe_angle = linkage_angle(rs00_q) + vehicle_pitch
```

If linkage backlash or support compliance exceeds the calibration budget, add
a direct angle encoder at hinge C. Do not treat the chassis IMU as a pipe-angle
sensor.

## Communication packet

Recommended Raspberry Pi to PSOC payload:

```text
sync | protocol_version | sequence | capture_time_us
     | ball_x_mm | confidence | processing_age_ms | status | CRC16
```

Do not transmit ASCII floating-point values in the control path. Use a
fixed-size binary packet, explicit little-endian fields, CRC16, sequence
checking and a receive timeout.

Recommended MSPM0 to PSOC payload:

```text
sync | sequence | imu_capture_time_us | pitch | pitch_rate
     | axial_acceleration | imu_status | CRC16
```

Recommended degradation:

| Time without valid camera | Action |
|---|---|
| <100 ms | observer prediction; freeze integration if actuator is saturated |
| 100–250 ms | restrict vehicle acceleration and pipe command to ±5 degrees |
| >250 ms | command controlled vehicle stop; enter ball-retention mode |
| IMU source stale >100 ms | stop vehicle acceleration; reject feedforward |
| RS00 stale >10 ms | PSOC stops motion demand and enters motor-safe state |

## Vehicle speed and map

The steady axial acceleration that can be cancelled by pipe tilt is bounded by:

```text
a_cancel ~= g*tan(available_pipe_angle)
```

At 6 degrees this is about 1.03 m/s² before reserving angle for feedback and
vehicle pitch. Normal route planning should reserve about 2 degrees for ball
feedback, so the planned acceleration should generally be no more than roughly
0.4–0.6 m/s² until road tests identify a tighter bound.

For a braking distance `distance`:

```text
speed <= sqrt(2 * allowed_acceleration * distance)
```

For a segment in which the vehicle must both accelerate and decelerate
symmetrically:

```text
peak_speed <= sqrt(allowed_acceleration * segment_length)
```

Replace these symbols with the competition's actual map dimensions, target
speed, stop-zone length, turn radius and slope before final gain/trajectory
tuning.

## Current measured-source-rate simulation result

The existing stress-test `Pass` field is a mechanical-safety result:

- ball not dropped;
- linkage remains valid;
- distance to pipe end at least 25 mm;
- peak ball/pipe slip no more than 150 mm/s.

It is not the competition's 10 mm accuracy result. The recovery test starts
with a 25 mm position error, so strict all-time 10 mm accuracy is false by
construction. Competition reporting must separately state the settling
interval and then require the peak absolute position error to stay at or below
10 mm during the specified scoring interval.

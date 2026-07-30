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
  ├─ 500 Hz IMU acquisition and attitude/acceleration estimate
  └─ timestamped IMU packet + CRC
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
2. ±8 degree pipe-angle limit;
3. 1.2 rad/s pipe command-rate limit;
4. linkage inverse kinematics;
5. RS00 position/speed/torque/temperature limits.

The disturbance observer should remain slower than the delayed camera path.
A high-bandwidth standalone LADRC will interpret delayed position as a new
disturbance and can produce a limit cycle on the slippery pipe.

## Real-time task budget

The MSPM0 IMU period is 2 ms. It timestamps the data-ready event, performs only
the required attitude/axis preprocessing and sends a fixed-size packet to the
PSOC.

| Device/task | Suggested method |
|---|---|
| MSPM0, 500 Hz | IMU SPI + DMA; timestamp at data-ready interrupt |
| MSPM0, 500 Hz | gyro/accelerometer preprocessing and CRC packet |
| PSOC, 500 Hz | consume latest timestamped IMU and RS00 CAN feedback |
| PSOC, 200 Hz | 3-state observer, LQI, feedforward and edge supervisor |
| PSOC, 500 Hz | linkage conversion, command interpolation and safety limits |
| PSOC, 500 Hz | pre-built RS00 CAN frame and transmit-deadline watchdog |

Run the position observer/LQI at 200 Hz, but update the IMU state, motor
feedback and RS00 safety layer at 500 Hz on the PSOC. A new camera sample is
used only once, according to frame ID and capture timestamp.

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
| IMU stale >10 ms | stop vehicle acceleration; reject feedforward |
| RS00 stale >10 ms | PSOC stops motion demand and enters motor-safe state |

## Vehicle speed and map

The steady axial acceleration that can be cancelled by pipe tilt is bounded by:

```text
a_cancel ~= g*tan(available_pipe_angle)
```

At 8 degrees this is about 1.38 m/s² before reserving angle for feedback and
vehicle pitch. Normal route planning should reserve at least 3–4 degrees for
ball feedback, so the planned acceleration should generally be lower than the
raw 8-degree limit.

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

## Current 500 Hz simulation result

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

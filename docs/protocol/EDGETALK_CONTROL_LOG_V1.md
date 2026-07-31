# EdgeTalk control log V1

V1只保留用于读取旧`.hblg`文件。当前固件发送的全量传感器和独立时间戳格式见
[`EDGETALK_CONTROL_LOG_V2`](EDGETALK_CONTROL_LOG_V2.md)，树莓派解码器同时兼容V1/V2。

This is a read-only USB CDC telemetry frame. It never carries actuator
commands. The M33 guard publishes a latest-value snapshot at at most 50 Hz;
the USB worker performs the actual transfer.

All integers and IEEE-754 floats are little-endian. Each frame is 80 bytes:

| Offset | Type | Field |
|---:|---|---|
| 0 | char[4] | `HBLG` |
| 4 | u16 | version = 1 |
| 6 | u16 | frame size = 80 |
| 8..36 | integers | log/time/sensor/control sequence, valid/mode/guard/status |
| 40..72 | 9 x f32 | measured x, estimated x/v/disturbance, pipe target, motor q/dq, longitudinal acceleration, body pitch |
| 76 | u32 | CRC32C over bytes 0..75 |

The Raspberry Pi bridge drains this return stream while sending 100 Hz vision
frames. Pass `--telemetry-log PATH` to append only CRC-valid binary frames.
At 50 Hz the raw rate is 4 kB/s; rotate or stop collection during long runs.

## Q3 actual-control extension

`status_flags` reserves:

- bit 16 `Q3_ACTUAL`: this record comes from the deployed Q3 PID, not M55 shadow;
- bit 17 `CONTROL_ACTIVE`;
- bit 18 `Q3_PASSED`.

For `Q3_ACTUAL` records only:

- `control_mode = 0x0101`;
- `guard_reason` carries the Q3 phase;
- the legacy `estimated_disturbance_mps2` slot carries `target_position_m`.

The context-dependent slot keeps the V1 wire frame at 80 bytes. Consumers must
check `Q3_ACTUAL` before interpreting it as a position target. While Q3 is
active, the USB latest-value queue gives actual-control records priority over
M55 shadow records.

Convert a raw capture for MATLAB/Simulink with:

```bash
python3 control_log_to_csv.py run.hblg run.csv
```

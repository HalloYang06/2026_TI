# EdgeTalk control log V2

V2是M33通过USB CDC发给树莓派的只读诊断日志，不携带任何执行器命令。M33最多
以50 Hz生成最新值快照；每帧288字节，约14.4 kB/s。树莓派只把CRC32C有效帧写入
`.hblg`，随后可转换为CSV供MATLAB/Simulink回放。

## 时间域

每个源同时保存源时间和M33接收时间，禁止把USB到达时间冒充采集时间：

| 字段 | 时间域 | 含义 |
|---|---|---|
| `produced_time_ms` | M33 | 日志快照生成时间 |
| `vision_capture_time_us` | 树莓派 | 相机采集时间 |
| `vision_receive_time_ms` | M33 | M33收到视觉帧时间 |
| `imu_source_time_ms` | MSPM0 | 完整IMU epoch提交时间 |
| `accel/gyro/attitude_receive_time_ms` | M33 | 三类CAN向量到达时间 |
| `imu_sync_receive_time_ms` | M33 | `0x104`源时间帧到达时间 |
| `wheel_receive_time_ms` | M33 | 轮速CAN帧到达时间 |
| `motor_receive_time_ms` | M33 | RS00反馈帧到达时间 |

树莓派、MSPM0和M33是独立单调时钟。离线分析必须保留源时间和接收时间，通过序号/
epoch和接收时间对齐；在时钟偏差同步完成前，不能直接用不同时间域相减计算单程延迟。

## 288字节帧

| 偏移 | 内容 |
|---:|---|
| 0..7 | `HBLG`、version=2、frame_size=288 |
| 8..35 | 日志/传感器/控制序号、状态位、模式和保护原因 |
| 36..55 | 视觉序号、采集时间、M33接收时间、处理耗时 |
| 56..87 | IMU epoch/mask、源时间、各CAN接收时间、各源序号 |
| 88..127 | 轮速/电机接收时间、设备状态、数据age和控制输出标志 |
| 128..283 | 39个float32，见下表 |
| 284..287 | bytes 0..283的CRC32C |

float32依次包括：钢球位置/置信度、估计位置/速度、目标位置/估计扰动、管角目标/实测、
电机目标/实测角度/速度/力矩/温度/电流/母线电压、三轴加速度、三轴角速度、roll/
pitch/yaw、左右轮速/车速，以及位置误差、积分、滤波加速度、反馈/前馈/请求/最终管角、
PID三项和控制器角度/速率限幅。

## 树莓派采集与转换

```bash
python3 edgetalk_camera_bridge.py \
  --telemetry-log ~/hball-logs/q4_001.hblg

python3 control_log_to_csv.py \
  ~/hball-logs/q4_001.hblg ~/hball-logs/q4_001.csv
```

默认CSV只保留Q3～Q6实控帧；`--all-sources`同时保留M55 shadow，`--q3-only`只保留
Q3。原始日志、CSV和视频属于实测数据，不提交Git。

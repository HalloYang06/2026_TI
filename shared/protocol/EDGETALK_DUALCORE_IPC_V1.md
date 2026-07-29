# EDGETALK_DUALCORE_IPC_V1：H题双核共享快照

## 用途与边界

EdgeTalk M33独占USB、CAN和最终安全门，M55只运行200 Hz状态估计/LQG和10 Hz LVGL。两核通过一个固定256字节的共享区交换：

- `sensor`：160字节，M33单写、M55单读；
- `control`：96字节，M55单写、M33单读。

本协议不复用参考机械臂工程的native enum/union消息。所有多字节字段显式小端编码，浮点为IEEE-754 `float32`，结构体只作为对齐后的字节容器。版本1的M55输出必须带`SHADOW_ONLY`，M33不得将其接入CAN发送。

## 内存一致性

共享区和两个槽都按32字节cache line对齐，槽长也是32字节的整数倍。每个槽偏移0的`seqlock`不参与CRC：

1. 写端把`seqlock`改为奇数，执行内存屏障并clean首个cache line；
2. 写端更新偏移4之后的帧、CRC和保留区，执行屏障并clean整个槽；
3. 写端把`seqlock`改为下一个偶数，执行屏障并再次clean首行；
4. 读端先invalidate整个槽并读取偶数`seqlock`，复制帧后invalidate首行并再次读取；
5. 两次序号不等或任一为奇数时返回`BUSY`，由下一控制周期重试；稳定后再验头、CRC和数值范围。

平台适配必须提供真实的D-cache clean/invalidate和硬件memory barrier；主机测试的空钩子只验证编码与状态机，不能证明双核cache一致性。

## 公共32字节头

| 偏移 | 字节 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 4 | `seqlock:u32` | 奇数写入中，偶数稳定；不参与CRC |
| 4 | 4 | `magic:u32` | `0x50494248`，内存字节为`HBIP` |
| 8 | 2 | `version:u16` | 固定1 |
| 10 | 2 | `message_type:u16` | 1传感器，2控制shadow |
| 12 | 2 | `frame_size:u16` | 160或96 |
| 14 | 2 | `payload_size:u16` | 96或32 |
| 16 | 4 | `producer_sequence:u32` | 传感器快照序号或控制器step |
| 20 | 4 | `produced_time_ms:u32` | 生产核单调毫秒时刻 |
| 24 | 4 | `valid_flags:u32` | 传感器valid或控制flags |
| 28 | 4 | `reserved:u32` | 版本1必须为0 |

## sensor帧

payload位于帧偏移32，固定96字节。CRC32C位于156，覆盖偏移4..155；未使用字节必须写0。

| 帧偏移 | 字节 | 字段 |
|---:|---:|---|
| 32 | 4 | `vision_sequence:u32` |
| 36 | 2 | `accel_sequence:u16` |
| 38 | 2 | `gyro_sequence:u16` |
| 40 | 2 | `attitude_sequence:u16` |
| 42 | 2 | `wheel_sequence:u16` |
| 44 | 2 | `msp_status_flags:u16` |
| 46 | 1 | `motor_fault_summary:u8` |
| 47 | 1 | `reserved:u8` |
| 48 | 8 | `vision_capture_time_us:u64` |
| 56 | 4 | `vision_receive_age_ms:u32` |
| 60 | 4 | `imu_age_ms:u32` |
| 64 | 4 | `wheel_age_ms:u32` |
| 68 | 4 | `motor_age_ms:u32` |
| 72 | 4 | `heartbeat_age_ms:u32` |
| 76 | 4 | `ball_position_m:f32` |
| 80 | 4 | `vision_confidence:f32` |
| 84 | 4 | `longitudinal_accel_mps2:f32` |
| 88 | 4 | `lateral_accel_mps2:f32` |
| 92 | 4 | `body_pitch_rad:f32` |
| 96 | 4 | `yaw_rate_rad_s:f32` |
| 100 | 4 | `body_speed_mps:f32` |
| 104 | 4 | `motor_angle_rad:f32` |
| 108 | 4 | `motor_velocity_rad_s:f32` |
| 112 | 4 | `motor_torque_nm:f32` |
| 116 | 12 | 保留，写0 |
| 128 | 28 | cache-line填充，写0 |
| 156 | 4 | `crc32c:u32` |

头中的`producer_sequence/produced_time_ms/valid_flags`分别还原为传感器快照的`sequence/created_time_ms/valid_flags`。

## control shadow帧

payload位于帧偏移32，固定32字节。CRC32C位于92，覆盖偏移4..91。

| 帧偏移 | 字节 | 字段 |
|---:|---:|---|
| 32 | 4 | `source_sensor_sequence:u32` |
| 36 | 4 | `controller_steps:u32` |
| 40 | 4 | `deadline_misses:u32` |
| 44 | 2 | `mode:u16`，0..4 |
| 46 | 2 | `flags:u16` |
| 48 | 4 | `target_angle_rad:f32` |
| 52 | 4 | `estimated_position_m:f32` |
| 56 | 4 | `estimated_velocity_mps:f32` |
| 60 | 4 | `estimated_disturbance_mps2:f32` |
| 64 | 28 | 保留，写0 |
| 92 | 4 | `crc32c:u32` |

`flags`位0为`SAFETY_ELIGIBLE`，只表示M55算法输入满足条件；位1为强制的`SHADOW_ONLY`。即使位0为1，M33仍必须检查输出新鲜度、单调序号、有限值、急停、CAN/IMU/电机健康、角度限幅和变化率，且当前固件永久禁止执行器发送。

## 校验与失效规则

- CRC使用CRC-32C/Castagnoli，参数与视觉64字节帧相同；`"123456789"`为`0xE3069283`。
- 任意NaN/Inf、视觉置信度不在`[0,1]`、控制mode大于4、未知控制flags或缺少`SHADOW_ONLY`均返回`RANGE`。
- 未发布的全零槽返回`HEADER`；写入中或读到撕裂序号返回`BUSY`；均不得沿用为新控制输入。
- `u32`序号和毫秒时刻按模运算处理回绕。M33对控制输出的新鲜度应优先按本地观察到新`controller_steps`的时刻计算，不能假设两套RT-Thread tick零点完全一致。

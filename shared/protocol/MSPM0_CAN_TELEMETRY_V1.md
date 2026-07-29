# MSPM0_CAN_TELEMETRY_V1：底盘传感器 CAN 上报

## 边界

天猛星 MSPM0G3507 保持 IMU 采样、姿态解算、轮速/差速和底盘保护的本地实时闭环，通过 `1 Mbps Classic CAN` 向 EdgeTalk M33 上报状态。此版本只冻结 MSPM0 → M33 的只读遥测；M33 → MSPM0 的运动参考仍未启用。

所有帧使用 11 位标准 ID、数据帧、DLC 8，负载整数为小端。向量值采用 `int16` 毫单位，避免 CAN 上直接发送编译器相关浮点结构。

## 帧定义

| ID | 频率 | 字节0..1 | 字节2..3 | 字节4..5 | 字节6..7 |
|---:|---:|---|---|---|---|
| `0x080` | 20 Hz | heartbeat sequence `u16` | status flags `u16` | uptime_ms `u32` |
| `0x100` | 200 Hz | sample sequence `u16` | ax `i16`, mm/s² | ay `i16`, mm/s² | az `i16`, mm/s² |
| `0x101` | 200 Hz | sample sequence `u16` | gx `i16`, mrad/s | gy `i16`, mrad/s | gz `i16`, mrad/s |
| `0x102` | 100 Hz | sample sequence `u16` | left wheel `i16`, mm/s | right wheel `i16`, mm/s | body speed `i16`, mm/s |
| `0x103` | 100 Hz | attitude sequence `u16` | roll `i16`, mrad | pitch `i16`, mrad | yaw `i16`, mrad |

`0x100`和`0x101`应来自同一 IMU 采样并使用相同 sequence。姿态可以降低到100 Hz，但其采集时刻必须和本地姿态解算一致。EdgeTalk 不根据 CAN 到达顺序猜测传感器时间。

## 心跳状态位

| 位 | 名称 | 置位含义 |
|---:|---|---|
| 0 | `ESTOP_ACTIVE` | MSPM0本地急停或底盘禁止运动 |
| 1 | `IMU_VALID` | IMU初始化和自检通过 |
| 2 | `CHASSIS_READY` | 底盘本地闭环可运行，不等于已运动 |
| 3 | `LOCAL_CONTROL_ACTIVE` | MSPM0正在执行本地循迹/速度闭环 |

任何未知位按故障诊断保存，但不能自动解锁执行器。

## 时效与异常

- IMU/角速度目标200 Hz；`20 ms`无新数据即 stale。
- 姿态目标100 Hz；第一版同样按`20 ms`严格判定，实测调度抖动后可放宽到不超过`30 ms`。
- 轮速目标100 Hz；`30 ms` stale。
- 心跳目标20 Hz；`100 ms` stale。
- 扩展帧、远程帧、错误DLC和未知ID不能覆盖最新状态；分别累计 ignored/invalid 计数。
- sequence回绕按模运算处理。序号空洞只表示链路或源端丢样，禁止用上一帧伪造新采样。

## CAN负载

五类帧合计约620帧/s。按每个Classic CAN帧最坏约150 bit估算，占1 Mbps总线约`9.3%`。与RS00 250–500 Hz反馈和诊断帧叠加后仍有余量；实物验收必须同时观察TEC/REC、bus-off、FIFO lost/full和各消息age。

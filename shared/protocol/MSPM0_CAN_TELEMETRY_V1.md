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
| `0x103` | 200 Hz | attitude source sequence `u16` | roll `i16`, mrad | pitch `i16`, mrad | yaw `i16`, mrad |

`0x100`、`0x101`和`0x103`的字节0..1是源传感器帧序号，不是CAN发送次数。
MSPM0可以按200 Hz镜像最近值，但只有收到新的WIT `0x51/0x52/0x53`帧时相应序号
才允许变化。EdgeTalk把重复序号视为同一源样本，重复镜像不能刷新freshness。

理想情况下三类WIT帧属于同一个采样周期，三个序号应一致；任一类丢帧时允许暂时
不一致，但EdgeTalk不得把不一致的数据宣称为“同一时刻的完整IMU样本”。V1没有源端
采样时间字段，精确同周期绑定与采集时间见
[`MSPM0_CAN_TELEMETRY_V2_PROPOSAL`](MSPM0_CAN_TELEMETRY_V2_PROPOSAL.md)。

## 心跳状态位

| 位 | 名称 | 置位含义 |
|---:|---|---|
| 0 | `ESTOP_ACTIVE` | MSPM0本地急停或底盘禁止运动 |
| 1 | `IMU_VALID` | IMU初始化和自检通过 |
| 2 | `CHASSIS_READY` | 底盘本地闭环可运行，不等于已运动 |
| 3 | `LOCAL_CONTROL_ACTIVE` | MSPM0正在执行本地循迹/速度闭环 |

任何未知位按故障诊断保存，但不能自动解锁执行器。

## 时效与异常

- 加速度、角速度和姿态的CAN镜像目标均为200 Hz；源序号`20 ms`无变化即 stale。
- 轮速目标100 Hz；`30 ms` stale。
- 心跳目标20 Hz；`100 ms` stale。
- 扩展帧、远程帧、错误DLC和未知ID不能覆盖最新状态；分别累计 ignored/invalid 计数。
- sequence回绕按模运算处理。序号空洞只表示链路或源端丢样，禁止用上一帧伪造新采样。

## CAN负载

五类帧合计`20+200+200+100+200=720 frame/s`。按每个Classic CAN帧最坏约
150 bit估算，占1 Mbps总线约`10.8%`。与RS00 250–500 Hz反馈和诊断帧叠加后预计
仍有余量；实物验收必须同时观察TEC/REC、bus-off、FIFO lost/full、源序号变化率和
各消息age。

## 2026-07-30实现审计

- 当前WIT UART由SysConfig生成的波特率是`9600 bit/s`。三类11字节WIT帧在8N1下
  每个完整样本组至少需要330 bit，因此理论上限只有约`29.1`个完整组/s，不可能产生
  200 Hz或500 Hz新样本；720 frame/s只证明CAN镜像调度，不证明IMU源采样率。
- MSPM0发送代码已改为用`0x51/0x52/0x53`各自接收计数作源序号。升级前每次CAN发送
  都会增加序号，重复旧WIT值会被误判为新鲜数据。
- `IMU_VALID`现在要求加速度、角速度、姿态三类都出现且分别未超过100 ms；EdgeTalk
  仍按更严格的20 ms源序号age判定控制有效。
- 轮周长仍为0，`0x102`当前安全地上报全0；完成轮径、每圈计数和左右极性标定前，
  不能把它用于车速前馈。
- 急停输入尚未映射，心跳保持`ESTOP_ACTIVE=1`；这是禁止闭环解锁的安全锁，不是
  可以绕过的测试障碍。
- WIT解析仍在UART中断中进行，SysTick中仍执行浮点换算和MCAN排队。部署前必须用
  GPIO/DWT测量最坏执行时间，并把超预算工作移到低优先级任务。

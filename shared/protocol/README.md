# 三板通信协议

## 目标

协议先冻结消息语义，再根据最终引脚选择 CAN、USB CDC 或有线以太网。控制量不使用ASCII日志传输；日志和控制通道必须分离。

树莓派到 EdgeTalk 的钢球视觉包已经冻结为固定 64 字节的 [VISION_MEASUREMENT_V1](VISION_MEASUREMENT_V1.md)。该帧使用 CRC32C 和 `u32` 序号，替代下述通用草案的 CRC16/`u16` 序号；通用包头仍供 MSPM0 等尚未冻结的消息参考。

## 通用包头

第一版建议采用小端定长头：

| 字段 | 类型 | 说明 |
|---|---|---|
| magic | `u16` | 固定 `0x5AA5`，用于UART重同步 |
| version | `u8` | 协议版本，初始为1 |
| message_type | `u8` | 消息类型 |
| payload_length | `u16` | 负载字节数 |
| sequence | `u16` | 每个源独立递增，允许回绕 |
| capture_time_us | `u32` | 源端单调采集时间，按模运算处理回绕 |
| payload | bytes | 类型化负载 |
| crc16 | `u16` | 覆盖版本至payload，不覆盖magic |

在 UDP 上传输时仍保留序号、采集时间戳和CRC，方便统一回放与故障注入。CRC多项式和测试向量在固件实现前通过ADR冻结。

## 初始消息类型

| 类型 | 方向 | 频率 | 最小内容 |
|---|---|---:|---|
| `CHASSIS_STATE` | MSPM0 -> EdgeTalk | 200 Hz | `ax, ay, pitch, yaw_rate, wheel_l, wheel_r, status` |
| `BALL_MEASUREMENT` | 树莓派 -> EdgeTalk | 120 Hz | 固定64字节；圆心、半径、米制位置、质量、ROI、曝光和采集时间 |
| `BALL_TARGET` | 裁判/人机接口 -> EdgeTalk | 事件触发 | 目标位置、命令序号、有效期 |
| `CONTROL_HEALTH` | EdgeTalk -> MSPM0/显示 | 20~50 Hz | tracking、视觉龄期、IMU龄期、饱和、故障、降速请求 |
| `TIME_SYNC_REQ/RSP` | EdgeTalk <-> 各板 | 2~10 Hz | 四时间戳握手或往返时延样本 |

浮点负载便于原型阶段对拍，但正式 UART 包可改为带单位的定点数。任何单位变化都必须提升协议版本，禁止只靠注释猜测。

## 数据有效性

- 接收端先检查长度、版本、CRC、序号和时间戳，再写入共享状态。
- 乱序包只用于诊断，不能覆盖更新状态。
- 重启后的序号复位必须伴随 boot/session ID变化，避免把旧包当新包。
- 所有状态保存 `age` 和 `valid/stale`；控制器不能只依据“最近变量里有数值”。
- 视觉失锁、电机故障和急停是锁存或需明确恢复的状态，不因下一帧普通数据到达自动清除。

## 带宽估算

假设 `CHASSIS_STATE` 总长32字节，200 Hz在8N1 UART上约需64 kbit/s；`115200`虽能承载平均流量，但在重发、时钟误差和调试流量下余量偏小，因此首选 `460800` 或 `921600 bit/s`。控制UART禁止混入 `printf`。

视觉帧 64 字节，在 120 Hz 为 `7.68 kB/s`，在 240 Hz 验收档为 `15.36 kB/s`；当前 High-Speed USB CDC 有充分带宽余量。只传视觉测量，不传灰度 ROI 或完整图像。

## 待引脚确认后补充

- 各链路物理接口、电平、接插件、方向和引脚。
- CRC多项式、结构体精确布局、缩放单位和golden vectors。
- EdgeTalk与各板时钟同步方法及最大允许偏差。
- 每类消息的软/硬超时、降级动作和恢复条件。

# ADR-002：MSPM0+RS00 CAN、NanoPi USB 与 EdgeTalk 双核边界

## 状态

采用；运动发送仍保持关闭，等待传感器引脚、协议联调和人工台架验收

## 日期

2026-07-29

## 背景

最终候选硬件为天猛星 MSPM0G3507、NanoPi-M5、Infineon EdgeTalk 和机械臂拆下的 5 号灵足 RS00。用户决定 MSPM0 与 EdgeTalk 使用 CAN，NanoPi-M5 与 EdgeTalk 使用 USB，并让 EdgeTalk M33 负责 CAN、M55 负责部分算法和 LVGL。该决定取代 ADR-001 中“MSPM0 UART + 树莓派视觉 + EdgeTalk 单核控制”的早期划分。

## 决策

### 拓扑和所有权

```text
IMU/编码器/红外
       |
       v
MSPM0G3507 -- 1 Mbps Classic CAN --+--> EdgeTalk M33 <== shared IPC ==> EdgeTalk M55
  底盘本地闭环                        |    CAN/USB/安全门         LQG/LVGL
                                     |
RS00 5号电机 -- 1 Mbps Classic CAN --+

NanoPi-M5 -- USB CDC（二进制协议） --> EdgeTalk M33
  相机/钢球检测/采集时间戳
```

- MSPM0 保留红外循迹、轮速/差速和底盘保护的硬实时闭环。EdgeTalk 失联时，MSPM0 必须能独立停止底盘。
- NanoPi-M5 只做相机采集、钢球检测、标定和带采集时间戳的位置输出，不做最终执行器控制。
- M33 是唯一 CAN 和 USB 控制面所有者：接收 MSPM0/RS00/NanoPi 数据、统一时间戳、维护 freshness、发布 M55 输入快照，并对 M55 输出做最终安全检查。
- M55 以 200 Hz 运行延迟 Kalman、5帧球速拟合、扰动前馈和 LQG；以 10 Hz 更新 H 题专用 LVGL 页面。M55 不直接访问 CAN，不直接发送执行器命令。
- RS00 内部驱动器继续承担电流/FOC；EdgeTalk 不重复实现 20 kHz 三相 FOC。
- F407、树莓派和其他机械臂子系统不进入正式链路。

### CAN 总线

- 物理层统一为 `1 Mbps Classic CAN`，MSPM0 和 RS00 共线，标准帧与扩展帧并存。
- 总线两端各 `120 ohm`，断电测得约 `60 ohm`；所有节点共地，短支线，M33 仅有一个 RX 所有者。
- RS00 保留私有 29 位扩展 ID。5号电机 Get_ID 请求为 `0x0000FD05`，回复为 `0x000005FE`。
- MSPM0 使用 11 位标准 ID，初版分配如下；具体数据缩放在引脚和传感器确定后冻结：

| ID | 方向 | 频率 | 内容 |
|---|---|---:|---|
| `0x080` | MSPM0 -> M33 | 20 Hz | 心跳、故障、急停/底盘状态 |
| `0x100` | MSPM0 -> M33 | 200 Hz | 序号、`ax/ay/az` 定点量 |
| `0x101` | MSPM0 -> M33 | 200 Hz | 同序号、`gx/gy/gz` 定点量 |
| `0x102` | MSPM0 -> M33 | 100 Hz | 轮速、车速、转向/里程状态 |
| `0x120` | M33 -> MSPM0 | 100 Hz上限 | 经安全门批准的底盘参考；当前禁用 |

按每帧最坏约 150 bit 估算，IMU 400帧/s、底盘200帧/s、RS00反馈500帧/s、未来命令200帧/s和心跳约20帧/s，总占用约 `20%`，留有重发和调试余量。低 ID 的健康/故障帧具有更高仲裁优先级。

### NanoPi USB

- NanoPi-M5 为 USB Host，EdgeTalk 为 USB Device；第一版采用 CDC ACM，Linux 端表现为 `/dev/ttyACM*`。
- 使用 COBS 分帧、`0x00` 包尾、CRC32C、版本、消息类型、长度和单调序号；禁止以换行 JSON 进入 100 Hz 控制链。
- 视觉包包含 NanoPi 单调采集时间、钢球位置（米）、原始像素坐标、置信度和检测标志。只传测量，不传完整图像。
- 每秒执行一次四时间戳 ping/pong，估计 NanoPi 与 M33 的时钟偏移和 RTT；M33 根据采集时间计算视觉 age，而不是用 USB 接收时刻代替。
- 目标视觉频率为 `60~100 Hz`。USB CDC 断流超过 `100 ms` 即将视觉标为 stale。

### M33 与 M55 IPC

- 使用两个 32-byte cache-line 对齐的共享快照：`sensor_snapshot`（M33写/M55读）和 `control_shadow`（M55写/M33读）。
- 每个快照包含 magic、版本、结构体大小、单调序号、采集时间、valid flags 和 CRC；采用奇偶序号 seqlock，写入后 clean cache，读取前 invalidate cache。
- `sensor_snapshot` 至少包含：纵/横向加速度、俯仰、偏航角速度、底盘速度、钢球位置/age/置信度、RS00角度/速度/故障，以及各链路 age。
- `control_shadow` 包含：目标摆角、估计球位置/速度/等效扰动、控制器状态和诊断计数。
- M33 只接受单调新序号且 age `<=15 ms` 的输出，并再次执行有限值检查、`+-4 deg` 角度限幅和 `80 deg/s` 变化率限制。当前阶段输出永久 shadow，不连接 CAN 发送。

### 频率和超时

| 环节 | 频率/阈值 | 所属 |
|---|---:|---|
| JY901S UART三类报告 | 200 Hz（实测各约199.6 Hz） | MSPM0 |
| 底盘速度/差速闭环 | 1 kHz | MSPM0 |
| IMU CAN发布 | 200 Hz，stale `20 ms` | MSPM0 -> M33 |
| 车体/轮速发布 | 100 Hz，stale `30 ms` | MSPM0 -> M33 |
| RS00反馈 | 250~500 Hz，stale `20 ms` | RS00 -> M33 |
| NanoPi视觉 | 60~100 Hz，stale `100 ms` | NanoPi -> M33 |
| M33统一快照 | 200 Hz | M33 -> M55 |
| Kalman/LQG | 200 Hz | M55 |
| M55 shadow返回 | 200 Hz，stale `15 ms` | M55 -> M33 |
| M33安全监督 | 1 kHz | M33 |
| LVGL刷新数据 | 10 Hz | M55 |

### LVGL 页面

只显示本题所需状态：钢球位置/速度、IMU加速度、偏航角速度、RS00角度、LQG目标角、CAN RX、视觉age、控制周期/超期计数和 `SHADOW / TX OFF` 安全状态。不加入机械臂关节、EMG、康复、语音或其他历史工程页面。

### 降级和启用门

- CAN bus-off、MSPM0心跳丢失、IMU stale、RS00 stale、M55输出 stale 或非有限值：M33 禁止产生新电机/底盘命令。
- 视觉短时丢失只允许观测器预测；超过 `100 ms` 后请求底盘减速/停止并使摆杆回中，不用旧位置无限控制。
- 自动构建和自动测试永远不发送 enable、set-zero、位置、速度或力矩帧。新增运动路径必须经过人工确认车轮状态、限流电源、急停和断电接管。

## 当前实物证据

- M33 CAN-only 固件 `0.2.0-m33-can-only` 已于 2026-07-29 烧录。
- OpenOCD 确认 SMIF bank 位于 `0x60000000`；镜像写入 212,992 bytes，验证 210,012 bytes。
- 5号 RS00 Get_ID 完成 TX，收到扩展帧 `0x000005FE`；`TXBTO=1`、`TXBRP=0`、`ECR=0`，所有TX/RX/FIFO错误计数为0。
- 设备唯一标识只在临时串口日志中观察，不写入 Git。

## 后果

- M33 和 M55 不再共享驱动所有权，调试可以分别判断 CAN/USB、IPC和算法层。
- NanoPi-M5 替代树莓派进入正式视觉链路；后续相机和USB接口均只维护一套。
- 在实际 IPC 数据进入前，M55 LQG和LVGL仅能作为零输入 shadow 自检，不能据此宣称闭环完成。
- 引脚、MSPM0传感器型号、RS00主动反馈周期和NanoPi相机帧率确定后，需要冻结协议缩放和重新做总线负载测量。

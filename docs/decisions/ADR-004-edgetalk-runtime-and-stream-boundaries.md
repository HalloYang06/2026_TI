# ADR-004：冻结 EdgeTalk 双核运行时与 120 Hz 数据边界

## 状态

采用；所有控制输出保持 `SHADOW_ONLY`，运动发送未启用

## 日期

2026-07-30

## 背景

ADR-002 确定了 MSPM0、视觉板、EdgeTalk M33/M55 和 RS00 的职责，但其中的视觉板选型、`60~100 Hz` 目标、COBS 分帧和 M55 运行时仍是早期假设。当前已经完成 EdgeTalk M33 官方 emUSB CDC 实机打通、固定 64 字节视觉协议、CAN 只读解析、M33/M55 共享快照和 LQG shadow 管线，需要冻结一套与现有代码一致的边界。

本 ADR 取代 ADR-002 中关于视觉主机、USB 线协议、视觉频率和 M55 操作系统的相应部分；CAN 物理拓扑和三板职责继续沿用 ADR-002。

## 决策

### 板卡和运行时

- 当前正式视觉主机使用已打通链路的树莓派；NanoPi-M5 保留为性能或接口不达标时的单机替代，不让两块 Linux 板同时进入控制链。
- EdgeTalk M33 继续运行 RT-Thread，独占 USB Device、Classic CAN、输入时间新鲜度、双核 IPC 发布和最终安全门。
- EdgeTalk M55 使用 Infineon 官方 FreeRTOS `release-v10.6.202`，固定提交 `8a19c8db81becf1e981a5f94630952160fddf8c5` 和官方 `COMPONENT_CM55/TOOLCHAIN_GCC_ARM` port。M55 不链接 CAN 驱动，也不发送执行器命令。
- F407 不进入本题正式链路。RS00 内部驱动器继续闭合 FOC，EdgeTalk 不重复实现三相电流环。

### 分层数据路径

```text
树莓派 ROI/轮廓/圆心
  -> 64 B BALL_MEASUREMENT_V1
  -> M33 emUSB 流解析、CRC/序号/age
  -> 200 Hz sensor_snapshot
  -> M55 200 Hz 预测、延迟视觉更新、5帧球速、LQG
  -> control_shadow(SHADOW_ONLY)
  -> M33 1 kHz 有限值/freshness/限幅/限速安全门
  -> ACTUATOR_TX=0

MSPM0/RS00 1 Mbps Classic CAN
  -> M33 ID/DLC/IDE/RTR/序号/状态位检查
  -> 同一个 200 Hz sensor_snapshot
```

应用层不得绕过 M33 输入层直接把 USB 或 CAN 缓冲区交给 M55。M55 只处理带 `valid_flags`、各源 age 和单调序号的不可变快照。

### USB 视觉边界

- 灰度 ROI、二值图、轮廓点集和录像留在树莓派，只上传圆心、半径、米制位置、置信度、ROI 元数据、曝光/处理时间、序号、采集时间戳和 CRC32C。
- `BALL_MEASUREMENT_V1` 是固定 64 字节小端帧，依靠 magic、版本、长度和 CRC32C 做流重同步；不再增加 COBS 包装。
- 正式相机目标为 `120 Hz`，有效负载仅 `7.68 kB/s`。USB/M33 接收按 `240 Hz`、`15.36 kB/s` 验收；`500 Hz`、`32 kB/s` 只作为合成压力档。
- Linux 串口写超时固定为 `20 ms`。发生背压时丢弃过期采集时隙，不排队补发旧球位置。
- M33 每次以 512 字节缓冲调用官方阻塞 `USBD_CDC_Receive(..., 0)`，再用跨读取流解析器处理拆包和粘包。64 字节测量逐帧立即提交，不等待凑满 USB High-Speed 最大包。
- 实物验收必须同时满足主机 `achieved_rate_hz >= 237.6`、`deadline_misses=0`，以及 M33 `usb_speed=2`、`vision_rate_x10>=2376`、接收增量相等、CRC/重复/乱序/空洞为零。

### CAN 到算法边界

初版发布频率和用途如下：

| 数据 | 频率 | 算法用途 | 失效门 |
|---|---:|---|---:|
| MSPM0 心跳/状态 | 20 Hz | 急停、IMU 自检、底盘状态 | 100 ms |
| `ax/ay/az` | 200 Hz | 纵横向扰动预测与前馈 | 20 ms |
| `gx/gy/gz` | 200 Hz | 偏航角速度和转弯离心项 | 20 ms |
| roll/pitch/yaw | 100 Hz | 车体俯仰补偿 | 20 ms |
| 左右轮/车速 | 100 Hz | 底盘状态、曲率和打滑诊断 | 30 ms |
| RS00 角度/速度/故障 | 250~500 Hz，待实测 | 摆杆反馈和执行器健康 | 20 ms |

每一类 MSPM0 帧独立维护 16 位序号。重复帧和乱序帧不得刷新接收时间或覆盖算法输入；向前跳号累计丢帧数；序号自然回绕允许；检测到 MSPM0 uptime 回退时清除所有旧流有效态。IMU 帧即使新鲜，也只有在心跳 `IMU_VALID` 置位时才进入 `HBALL_SENSOR_VALID_IMU`。

RS00 当前只读反馈没有应用层序号，依靠 CAN 控制器错误状态、接收时间和 20 ms freshness 保护。电机编码器到真实摆杆角的零点、方向和传动比尚未标定，任何未来运动发送实现都必须先增加并验证这层映射；参考机械臂仓库的零点和方向禁止复用。

### 电机控制模式

- 首版正式模式选RS00的CSP位置模式。M55的LQG输出摆杆目标角，M33以1 kHz执行有限值、freshness、`+-4 deg`限幅、`80 deg/s`限速和故障门，再以200 Hz写入`loc_ref`；RS00内部闭合位置环和FOC。
- CSP调试时必须同时设置保守的`limit_spd`和`limit_cur`，并重新标定本机构的零点、方向、传动比和机械限位。当前`ACTUATOR_TX=0`，仓库只实现只读反馈和shadow目标，不实现CSP发送。
- 如果实测CSP的5%~95%上升时间、相位滞后或死区无法满足仿真包络，再把MIT位置-速度阻抗模式作为备选，仍由M33做外层安全门。纯速度模式存在角度积分漂移，纯力矩/电流模式依赖准确负载、摩擦和失联模型，首版拒绝采用。

### 控制和显示频率

| 层 | 频率 | 当前输出 |
|---|---:|---|
| M33 USB/CAN 输入 | 事件驱动，1 ms CAN 轮询预算 | 只读状态 |
| M33 统一快照 | 200 Hz | M33 -> M55 |
| M55 LQG/观测器 | 200 Hz | `SHADOW_ONLY` |
| M33 安全监督 | 1 kHz | `actuator_tx_allowed=false` |
| M33 -> RS00 CSP参考 | 200 Hz，当前禁用 | `ACTUATOR_TX=0` |
| RS00位置环/FOC | 驱动器内部频率，待厂家资料/实测 | 内部闭环 |
| LVGL 数据钩子 | 10 Hz | H题参数页面 |

## 备选方案

### M55 继续使用 RT-Thread

已有适配源码可做主机契约检查，但临时构建曾错误生成 Cortex-M7/ARMv7E-M 镜像，不能证明 Cortex-M55 异常、FPU/MVE 和上下文切换正确。因此不作为正式 M55 部署运行时。

### 通过 USB 传灰度 ROI

120 Hz 的 `320x120` 灰度 ROI 已达 `4.608 MB/s`，还会把图像拷贝、Linux 调度和 M33 解析抖动引入控制链。控制只需要结构化测量，因此拒绝。

### 树莓派和 NanoPi 同时工作

会增加相机占用、时间同步、供电和故障仲裁，不增加当前闭环可观测量，因此拒绝。

## 后果

- USB、CAN、IPC、算法和安全门可以分别统计频率、错误和 age，定位问题时不再混层。
- 120 Hz 相机不会限制 200 Hz 模型预测，重复视觉序号只消费一次。
- FreeRTOS、CMSIS和async-transfer依赖已固定。离线单核ELF已证明180个源文件能以`-mcpu=cortex-m55+nomve`链接为ARMv8.1-M Mainline，但官方示例的共享段为`0x262FC000`，与当前M33的`0x261C0000`不一致；多核合并也仍缺匹配M33 NS HEX。因此它只算真CM55编译证据，不是可部署镜像。
- 真实 240 Hz 二进制 USB、MSPM0 多流 CAN、RS00 主动反馈和电机到摆杆映射仍需在断开运动能力的台架上逐项验收。

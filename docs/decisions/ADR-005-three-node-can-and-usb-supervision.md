# ADR-005：三节点 CAN 只读联调与树莓派 USB 开机监督

## 状态

采用；EdgeTalk/RS00 已实机通过，MSPM0 等待下载器接入后完成三节点验收

## 日期

2026-07-30

## 背景

H题正式链路需要在同一条总线上同时承载 MSPM0 车体状态和 RS00 电机反馈，并让树莓派通过 EdgeTalk 的 USB Device Type-C 口持续提供视觉测量。首轮实机联调必须能够证明物理层、位时序、标准/扩展帧共存和开机重连，但不能因为自动测试产生任何运动命令。

## 决策

### 三节点 CAN

- EdgeTalk M33、天猛星 MSPM0G3507 和 5号 RS00 共用一条 `1 Mbps Classic CAN`；关闭 CAN FD 和 BRS。
- 总线使用线型拓扑、短支线、共地，两端各 `120 Ω`；断电后 CANH-CANL 目标约 `60 Ω`。
- RS00 保持 29 位扩展帧协议。自动线程只接收；唯一允许的发送是操作者从 FinSH 手工触发一次只读 Get_ID。自动探测固定关闭。
- MSPM0 使用 11 位、DLC 8、Classic CAN、小端定点遥测：

| ID | 内容 | 频率 |
|---|---|---:|
| `0x080` | 心跳、状态、uptime | 20 Hz |
| `0x100` | `ax/ay/az`，milli-m/s² | 200 Hz |
| `0x101` | `gx/gy/gz`，milli-rad/s | 200 Hz |
| `0x102` | 左右轮和车体速度，milli-m/s | 100 Hz |
| `0x103` | roll/pitch/yaw，milli-rad | 100 Hz |

- 每类 MSPM0 流使用独立 16 位序号。重复、乱序、跳号和重启分别计数；心跳未置 `IMU_VALID` 时，即使 IMU 数据新鲜也不进入算法有效态。
- MSPM0 当前没有任何 EdgeTalk→底盘控制帧或电机命令 API。轮周长未实测时，`0x102` 米制轮速输出 0；硬件急停未映射时，心跳始终置 `ESTOP_ACTIVE` 且不置 `CHASSIS_READY`。

### EdgeTalk 运行边界

- M33 集成镜像同时运行 emUSB、CAN 只读监视、200 Hz 统一快照和 1 kHz shadow 安全监督。
- `HBALL_BENCH_AUTO_PROBE5=0`、`MOTOR_COMMAND_TX=0`、`ACTUATOR_TX=0` 是硬边界。M55 shadow 不允许绕过 M33 安全门；当前 M55 可部署包尚未生成。
- CAN 状态必须保留原始最后一帧、TX pending/completed、TEC/REC、bus-off、FIFO full/lost和协议计数。`send queued` 不能替代 ACK/回复证据。

### 树莓派 USB 开机监督

- 树莓派运行用户级 systemd 服务 `hball-edgetalk-usb.service`，按“显式端口 -> 唯一 `/dev/serial/by-id` -> 唯一 `ttyACM`”发现 EdgeTalk。
- 服务以 pyserial 独占打开设备，并用 `flock` 防止第二个进程争抢串口；READY/PING 超时或 USB 断开后关闭并重新发现，`Restart=always`。
- 开机服务只发送换行分隔符和 `PING startup_safe=1`，不启动合成视觉流，不发送 `POSITION_VALID=1`，不访问 CAN。
- 用户 systemd 启用 linger，保证未登录也能开机启动。正式相机进程接管前必须先停止该独占守护；后续可把同样的发现、锁和重连机制复用到真实视觉服务。

## 实机证据

- EdgeTalk 的 SMIF bank 在写入前确认位于 `0x60000000–0x67FFFFFF`。集成 M33 镜像写入并校验 `339,968 bytes`，组合 XIP 校验 `332,708 bytes`，NS 校验 `230,252 bytes`，随后到达 Non-secure reset handler。
- M33 运行标签为 `0.3.0-m33-integrated-shadow`。CAN 为 1 Mbps，手工 Get_ID 完成一次 TX/ACK/回复；RS00 身份回复有效，设备唯一标识不写入 Git。TEC/REC、TX pending、bus-off和FIFO丢失均为0。
- 树莓派把 EdgeTalk 枚举为 `058b:0282` High-Speed CDC ACM并生成稳定 by-id路径。服务经树莓派真实重启后自动进入 `active`，收到新的 READY；EdgeTalk重刷导致USB暂时消失时也完成自动重连。
- 当前电脑没有检测到 MSPM0 DAPLink/CMSIS-DAP/XDS，也没有收到五类 MSPM0 帧。因此不能把两节点证据宣称为三节点完成。

## 后果

- 电机↔EdgeTalk 双向只读路径和树莓派↔EdgeTalk 开机链路已经独立验证；MSPM0 下载器是剩余的明确阻塞点。
- MSPM0 接入后只需烧录已构建镜像并检查五类帧、序号、频率和双方错误计数，不需要再修改 EdgeTalk 协议。
- 在 MSPM0 五类帧、真实 120 Hz 视觉和 M55共享地址分别通过前，统一快照保持无效，所有执行器发送继续关闭。

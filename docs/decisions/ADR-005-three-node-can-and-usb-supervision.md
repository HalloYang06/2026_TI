# ADR-005：三节点 CAN 只读联调与树莓派 USB 开机监督

## 状态

采用；三节点物理链路和 MSPM0 五类遥测已实机通过，连续 RS00 角度反馈与运动闭环仍未启用

## 日期

2026-07-30

## 背景

H题正式链路需要在同一条总线上同时承载 MSPM0 车体状态和 RS00 电机反馈，并让树莓派通过 EdgeTalk 的 USB Device Type-C 口持续提供视觉测量。首轮实机联调必须能够证明物理层、位时序、标准/扩展帧共存和开机重连，但不能因为自动测试产生任何运动命令。

## 决策

### 三节点 CAN

- EdgeTalk M33、天猛星 MSPM0G3507 和 5号 RS00 共用一条 `1 Mbps Classic CAN`；关闭 CAN FD 和 BRS。
- 总线使用线型拓扑、短支线、共地，两端各 `120 Ω`；断电后 CANH-CANL 目标约 `60 Ω`。
- RS00 保持 29 位扩展帧协议。自动线程只接收；唯一允许的发送是操作者从 FinSH 手工触发一次只读 Get_ID。自动探测固定关闭。
- MSPM0 使用 11 位、DLC 8、Classic CAN、小端定点遥测。`D0:D1` 均为该数据流独立递增的 `uint16` 序号：

| ID | D0:D1 | D2:D3 | D4:D5 | D6:D7 | 频率 |
|---|---|---|---|---|---:|
| `0x080` | sequence | status `uint16` | uptime low `uint16` | uptime high `uint16` | 20 Hz |
| `0x100` | sequence | `ax` `int16` | `ay` `int16` | `az` `int16` | 200 Hz |
| `0x101` | sequence | `gx` `int16` | `gy` `int16` | `gz` `int16` | 200 Hz |
| `0x102` | sequence | left `int16` | right `int16` | body `int16` | 100 Hz |
| `0x103` | sequence | roll `int16` | pitch `int16` | yaw `int16` | 200 Hz |

`0x100` 的缩放为 `0.001 m/s²/LSB`，`0x101` 和 `0x103` 为 `0.001 rad(/s)/LSB`，`0x102` 为 `0.001 m/s/LSB`，uptime 单位为 ms。status 位定义为：bit0 `ESTOP_ACTIVE`、bit1 `IMU_VALID`、bit2 `CHASSIS_READY`、bit3 `LOCAL_CONTROL_ACTIVE`，其余保留为 0。

姿态从 100 Hz 提高到 200 Hz，是因为仿真中的非线性前馈和 200 Hz LQG 每个 5 ms 周期都需要同步的 `ax/ay/pitch/gz`；保留 100 Hz 姿态会令俯仰成为最慢 IMU 分量。当前 MSPM0 总发送率为 `20+200+200+100+200=720 frame/s`。调度器每毫秒最多发送一帧，不把多个流挤在同一邮箱时刻。

按 11 位 Classic CAN、8 字节数据和最坏位填充估算，720 帧/s 约占 1 Mbps 总线 10%；即使后续 RS00 扩展反馈达到 500 Hz，总负载仍预计低于 20%。该估算不能替代最终示波器/逻辑分析仪总线占用测量。

- 每类 MSPM0 流使用独立 16 位序号。模 `2^16` 差值为 0 判重复，差值 `>=0x8000` 判乱序，其余大于 1 的差值累计 gap；心跳 uptime 明显回退时清空五类 valid 并记录重启。重复/乱序帧不刷新数据时间戳。
- CAN 控制器自身 CRC/ACK/错误状态负责单帧链路完整性；应用层不在 8 字节内重复放 CRC，而以 ID、IDE、RTR、DLC、独立序号和接收 age 做端到端质量门。IMU age 上限 20 ms、轮速 30 ms、心跳 100 ms；心跳未置 `IMU_VALID` 时，即使三类 IMU 帧新鲜也不进入算法有效态。未来若引入跨帧命令或分片，再单独定义应用 CRC 和事务序号。
- MSPM0 当前没有任何 EdgeTalk→底盘控制帧或电机命令 API。轮周长未实测时，`0x102` 米制轮速输出 0；硬件急停未映射时，心跳始终置 `ESTOP_ACTIVE` 且不置 `CHASSIS_READY`。
- MSPM0 进入 bus-off 后先等待 1000 ms，再通过 TI DriverLib 请求正常模式恢复；持续故障时最多每秒尝试一次，健康总线会重新武装下一次故障检测。恢复逻辑不绕过 CAN 控制器要求的隐性位观察过程。

### 算法输入与总线边界

- GitHub 中与本题匹配的仿真说明是 HalloYang06/2026_TI 提交 [`f9c4e025`](https://github.com/HalloYang06/2026_TI/commit/f9c4e02514610dbe63e42485d91af5f2c98e4113) 的 `experiments/h_ball_control_sim/LQR_MODEL.md`，运行边界后来由 [`fb9ad82`](https://github.com/HalloYang06/2026_TI/commit/fb9ad82ccd0563c43d8197481cf7c5b0dd78bf35) 更新。该提交已位于远端 `prep/2026` 的历史中。
- LQG 三状态为球位置误差、估计球速和摆杆相对前馈角误差；`ax/ay/pitch/gz` 进入非线性扰动前馈，RS00 实际角度进入执行器状态/跟随误差，轮速只作诊断和前馈校验，不硬塞进三状态 LQR。
- 树莓派的灰度 ROI/轮廓圆心结果保持走 USB：120 Hz 目标、64 字节 `BALL_MEASUREMENT_V1`，含位置、置信度、序号和采集时间戳；5 帧带时间戳拟合球速。视觉帧不在 CAN 上分片，避免增加仲裁负载、重组状态和不确定延迟。
- RS00 实际角度/速度反馈目标为 250~500 Hz、29 位扩展帧，沿用厂家协议并由 M33 解析；在厂家反馈周期或实测频率冻结前不重新定义其载荷。当前实机只验证了人工只读 Get_ID，未开启连续反馈或运动模式。

### EdgeTalk 运行边界

- M33 集成镜像同时运行 emUSB、CAN 只读监视、200 Hz 统一快照和 1 kHz shadow 安全监督。
- 当前 M33 使用 direct-PDL 接收路径；GFC 将未匹配标准帧和扩展帧都送入 FIFO0，实机 `msp_rx` 已证明 11 位帧可达。不要为本次协议改动 EdgeTalk 过滤器；仅当未来切回 RT-Thread CAN device 路径时，才修复其尚未实现的 `RT_CAN_CMD_SET_FILTER`。
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
- 更正接线后 EdgeTalk 同时收到 RS00 扩展帧和 MSPM0 标准帧；一次完整状态快照为 `msp_rx=437108`、`invalid=0`，heartbeat/accel/gyro/attitude/wheel 五类 valid 均为 1。
- 错接压力曾使 MSPM0 到达 bus-off（`ECR=0xF8`、`PSR=0x7E7`、TX pending=1），据此新增了 1 s 限频恢复状态机。首延迟、持续故障限频、健康后重武装和 32 位 ms 回绕四项主机测试通过；没有为了验证恢复而再次故意制造实车短路。
- 200 Hz 姿态镜像使用 Horco CMSIS-DAP 和 TI 官方 `TexasInstruments.MSPM0G1X0X_G3X0X_DFP 1.3.1` 烧录，pyOCD 明确报告擦除/写入 `46,080 bytes / 45 pages`。官方 pack SHA-256 为 `071BD317FC0F152DED6B2AE594D79C6FC5EB9952370526B4C14EF5B3B9807860`；固件 ELF/HEX SHA-256 分别为 `CDB39EDCF39835A6BA35158BBA429DFA7586EAEBC040E40CAF340DA26892AB93` 和 `D9AABCE54CCA0C215533CDF7F0C1C6BEB7368C6D15533985398B3231C7A50672`。
- 刷写后 14 s 只读日志中 `can_rx` 从 `1,486,327` 增至 `1,495,794`，按日志到达时间折算约 `729.1 frame/s`，与独立时钟误差下的 720 frame/s 目标一致；所有样本 `tx_fail=0`、`MOTOR_COMMAND_TX=0`、`ACTUATOR_TX=0`。本测试保持电机/底盘动力隔离、车轮悬空无载、可人工拔线接管。

## 后果

- 三节点总线的物理层、标准/扩展帧共存、MSPM0 五类遥测解析和 720 frame/s 调度已经验证，不需要为了 200 Hz 姿态再修改 EdgeTalk 过滤器。
- 尚未完成的是 RS00 连续 250~500 Hz 角度反馈、真实 120 Hz 视觉、M33/M55 共享地址统一和任何运动闭环；这些项完成前所有执行器发送继续关闭。
- 实测 IMU 型号/固件、CAN 收发器 EN/STB、轮周长、编码器 counts/rev 和 RS00 反馈周期仍需补入硬件参数文档；`0x102` 在这些标定完成前保持安全的 0 值。

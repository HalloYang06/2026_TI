# H题车载滚球控制交接

AI identity: Codex

Role: H-ball EdgeTalk USB/CAN/LQG integration

Updated: 2026-07-30

## 当前结论

正式控制链仍为“天猛星MSPM0G3507 + 树莓派 + EdgeTalk M33/M55 + RS00”。F407不进入；NanoPi-M5仅作树莓派性能或接口不达标时的单机备选；`PSOC_E84_robot`只提供工具链和RS00协议参考，不迁移机械臂业务、零点或运动参数。

本轮已经完成两条真实链路：

1. 树莓派与EdgeTalk USB CDC已部署开机守护并通过真实重启、EdgeTalk重刷断连和自动重连验收。
2. EdgeTalk与5号RS00在`1 Mbps Classic CAN`下完成一次人工只读Get_ID，TX/ACK/回复正常，所有CAN错误计数为0。

MSPM0协议、MCAN端口和可烧录镜像已完成，但当前电脑没有检测到MSPM0下载器，EdgeTalk仍为`msp_rx=0`。因此“电机-EdgeTalk-MSP三节点CAN”尚差MSPM0实板烧录与五类帧对拍，不能宣称全部完成。

当前分支为`prep/2026`，未创建或填充`main`。所有自动任务保持`MOTOR_COMMAND_TX=0`、`ACTUATOR_TX=0`，M55只允许`SHADOW_ONLY`。实机只发送过人工触发的一次RS00只读Get_ID；从未发送enable、set-zero、位置、速度或力矩。

## 当前数据路径

```text
树莓派120 Hz灰度ROI/轮廓/圆心
  -> 64 B BALL_MEASUREMENT_V1
  -> M33 emUSB流解析/CRC/序号/age

MSPM0五类标准帧 + RS00扩展反馈
  -> 1 Mbps Classic CAN
  -> M33质量门/200 Hz sensor_snapshot
  -> shared SRAM IPC
  -> M55 FreeRTOS 200 Hz观测器/LQG（尚不可部署）
  -> control_shadow(SHADOW_ONLY)
  -> M33 1 kHz安全监督
  -> ACTUATOR_TX=0
```

M33是USB、CAN、输入有效性和最终安全门的唯一所有者；M55不链接CAN或执行器发送。LVGL数据钩子为10 Hz，只显示本题参数和`SHADOW / TX OFF`。

## 本轮提交

- `05f052f feat(mspm0): define safe CAN telemetry contract`
- `0ea6e4f feat(mspm0): integrate safe MCAN telemetry port`
- `910d346 feat(raspberrypi): supervise EdgeTalk USB at boot`
- `ddd6c82 fix(edgetalk): label integrated shadow runtime`

MSPM0只新增CAN相关接口与构建接入：PA26=`CANFD0_CANTX`、PA27=`CANFD0_CANRX`，40 MHz MCAN时钟，1 Mbps、87.5%采样点，Classic CAN/FDF/BRS关闭。帧合同为：

| ID | 内容 | 频率 |
|---|---|---:|
| `0x080` | 心跳、状态、uptime | 20 Hz |
| `0x100` | 加速度，milli-m/s² | 200 Hz |
| `0x101` | 角速度，milli-rad/s | 200 Hz |
| `0x102` | 左右轮/车速，milli-m/s | 100 Hz |
| `0x103` | 姿态，milli-rad | 100 Hz |

每类流有独立16位小端序号。轮周长未实测，`0x102`当前安全输出0；硬件急停未映射，心跳始终置`ESTOP_ACTIVE`且不置`CHASSIS_READY`。MSPM0没有任何电机或底盘命令API。

## 实机与构建证据

### EdgeTalk M33

- 构建模式：`HBALL_USB_ONLY=0`、`HBALL_INTEGRATED_SHADOW=1`。
- 运行标签：`0.3.0-m33-integrated-shadow`。
- ELF：`text=214596 data=15656 bss=244516`。
- SHA-256：ELF `A68AA8435BFE361D1448706A13242D66746C41674AFD1936440979EA04ECCD4B`；NS HEX `5F449A2DB5AC486B59EEB9B537334F8D4522A11F809D30FE0627BD265E6732D2`；raw Secure+NS `E0C7733AAADAC9D2CD892A50529DD2896C1251259B1FCFA5CA8BB8581646C34D`；XIP verify `0DED6751F5F887E020F039E748A1756B983C44A3742AA72AD49C51AC7FC1F318`。
- OpenOCD预检确认`cat1d.cm33.smif1_ns`位于`0x60000000–0x67FFFFFF`。
- 写入/校验：raw `339,968 bytes`、组合XIP `332,708 bytes`、NS `230,252 bytes`，随后到达Non-secure reset handler。
- 运行日志持续输出200 Hz快照、1 kHz安全监督、`MOTOR_COMMAND_TX=0`和`ACTUATOR_TX=0`。

### RS00 CAN

- 手工只读Get_ID请求为扩展帧`0x0000FD05`，回复为`0x000005FE`；设备唯一标识已核对但不写入Git。
- `tx=1/1/0`、`TXBRP=0`、TEC/REC=0、bus-off=0、FIFO full/lost=0。
- 该结果只证明电机↔EdgeTalk双向链路，不代表电机已经使能或运动。

### 树莓派 USB

- EdgeTalk枚举为`058b:0282`、High-Speed `cdc_acm`，稳定by-id路径存在。
- 用户systemd服务：`hball-edgetalk-usb.service`，`enabled`、`active`，linger=`yes`。
- 树莓派真实重启后服务自动启动并收到READY；EdgeTalk两次重刷时服务先WAIT/RECONNECT，设备恢复后自动重新收到READY。
- 守护只发换行和PING，串口独占并带进程锁；不发合成有效球位置，不访问CAN。

### MSPM0

- 主机测试：`3 passed`；SysConfig警告按错误检查通过；ARM GCC完整链接通过。
- ELF：`D:\2026_TI\.codex_tmp\mspm0_gcc\build\wit-oled-hardware-spi.elf`。
- HEX：`D:\2026_TI\.codex_tmp\mspm0_gcc\build\wit-oled-hardware-spi.hex`。
- SHA-256：ELF `D0C4174B7EE388F652E850A2565668F5107BC35C549096E79431885715BF569A`；HEX `E860BAD623AFD2C0709FC4D0228F87CA4F8E294FA5EDB57D295ABC9EEA460EA8`。
- 当前电脑未检测到DAPLink/CMSIS-DAP/XDS，尚未烧录；EdgeTalk当前`msp_rx=0`。

### 软件回归

- `python -m pytest firmware/edgetalk/tests vision/raspberrypi/tests -q` -> `51 passed`。
- `python -m pytest firmware/mspm0/wit-oled-hardware-spi/tests -q` -> `3 passed`。
- 集成M33 SCons/ARM链接通过；树莓派USB守护测试为17项，包含by-id选择、独占、重连、PING-only和systemd安全合同。

## M55状态

官方FreeRTOS CM55临时工程已完成180/180源文件编译和单核链接，但仍不能烧录：临时工程`.hball_ipc_shared=0x262FC000`，当前M33为`0x261C0000`，两核地址不一致，且缺少匹配的多核后处理包。M33日志中的IPC读取失败/无shadow符合预期；在共享地址统一前禁止烧录M55或接入执行器。

## 下一步：完成三节点CAN

1. 接入天猛星的DAPLink/CMSIS-DAP/XDS，确认设备在电脑枚举；不要误用EdgeTalk KitProg3。
2. 台架保持底盘/电机动力隔离、轮子/执行器卸载或无负载、仅调试器/USB或限流电源供电、急停和人工拔线可用。
3. MSPM0 PA26/PA27必须经过3.3 V CAN收发器再接CANH/CANL；三节点共地，线型拓扑，两端120 Ω，断电测约60 Ω。
4. 烧录上面的MSPM0 HEX。调试器检查`g_hball_can_stats.initialized=1`、TX confirmed递增、TEC/REC=0、bus-off/FIFO lost=0、`rx_extended`能看到RS00链路。
5. EdgeTalk执行只读`hball_status`，验收`msp_rx`持续增加，心跳/加速度/角速度/姿态/轮速五类valid出现，序号duplicate/ooo/gap为0；同时保持`tx=0`，无需再次发Get_ID。
6. 记录IMU型号/固件、CAN收发器型号和EN/STB接法、轮周长与编码器counts/rev后，才能把轮速从0改为实测米制值。
7. 用逻辑分析仪或GPIO测量SysTick占用、WIT UART丢帧和实际620帧/s遥测；若ISR预算不足，把浮点/64位换算移到主循环，并给FIFO0排空增加单次预算。未来新增任何TX入口时，在最底层增加遥测ID白名单，不能只依赖`MOTOR_COMMAND_TX=0`标记。

## 工作树保护

以下未跟踪内容属于用户既有工作，本轮没有暂存或提交：

- `experiments/h_ball_control_sim/hballsim/full_vehicle_simulation.py`
- `experiments/h_ball_control_sim/hballsim/vehicle_campaign.py`
- `experiments/h_ball_control_sim/hballsim/vehicle_dynamics.py`
- `experiments/h_ball_control_sim/hballsim/vehicle_parameters.py`
- `experiments/h_ball_control_sim/tests/test_full_vehicle_*.py`
- `experiments/h_ball_control_sim/tests/test_vehicle_*.py`
- `firmware/edgetalk/config/`
- `.codex_tmp/`和`tmp/`

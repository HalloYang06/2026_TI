# H题车载滚球控制交接

AI identity: Codex

Role: H-ball EdgeTalk USB/CAN/LQG integration

Updated: 2026-07-30

## 当前结论

正式控制链仍为“天猛星MSPM0G3507 + 树莓派 + EdgeTalk M33/M55 + RS00”。F407不进入；NanoPi-M5仅作树莓派性能或接口不达标时的单机备选；`PSOC_E84_robot`只提供工具链和RS00协议参考，不迁移机械臂业务、零点或运动参数。

本轮已经完成三条真实链路：

1. 树莓派与EdgeTalk USB CDC已部署开机守护并通过真实重启、EdgeTalk重刷断连和自动重连验收。
2. EdgeTalk与5号RS00在`1 Mbps Classic CAN`下完成一次人工只读Get_ID，TX/ACK/回复正常，所有CAN错误计数为0。
3. 更正接线后，天猛星MSPM0G3507以11位标准帧持续发送五类遥测，EdgeTalk在同一总线上同时接收RS00扩展帧；五类解析有效，200 Hz姿态镜像实测总接收约729.1帧/s。

因此可以宣称“三节点CAN物理链路、标准/扩展帧共存和MSPM0遥测合同已打通”。不能宣称滚球运动闭环已完成：RS00目前只验证了只读Get_ID，连续250~500 Hz角度反馈、真实120 Hz视觉、M33/M55共享地址统一和执行器控制仍待完成。

当前分支为`prep/2026`，未创建或填充`main`。所有自动任务保持`MOTOR_COMMAND_TX=0`、`ACTUATOR_TX=0`，M55只允许`SHADOW_ONLY`。实机只发送过人工触发的一次RS00只读Get_ID；从未发送enable、set-zero、位置、速度或力矩。

## 当前数据路径

```text
树莓派120 Hz灰度ROI/轮廓/圆心
  -> 64 B BALL_MEASUREMENT_V1
  -> M33 emUSB流解析/CRC/序号/age

MSPM0五类标准帧 + RS00扩展帧（当前仅Get_ID；连续角度反馈待启用）
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
- `50dd0fa docs(bringup): record CAN and USB bench evidence`
- `22aea9a fix(mspm0): recover CAN telemetry after bus-off`

本题算法仿真说明来自GitHub提交[`f9c4e025`](https://github.com/HalloYang06/2026_TI/commit/f9c4e02514610dbe63e42485d91af5f2c98e4113)（`feat(sim): add H-problem LQG stress model`），边界说明由[`fb9ad82`](https://github.com/HalloYang06/2026_TI/commit/fb9ad82ccd0563c43d8197481cf7c5b0dd78bf35)更新；两者都位于远端`prep/2026`历史。当前结论是“可测扰动非线性前馈 + 五帧球速拟合/Kalman + 增广LQG”，不是把所有传感器塞进一个PID：

- `ax/ay/pitch/gz`进入车体运动扰动前馈；
- 球位置和估计球速进入外层LQG；
- RS00实际角度进入摆杆执行器状态和跟随误差；
- 轮速用于诊断/前馈校验，不增加三状态LQR维数。

MSPM0只新增CAN相关接口与构建接入：PA26=`CANFD0_CANTX`、PA27=`CANFD0_CANRX`，40 MHz MCAN时钟，1 Mbps、87.5%采样点，Classic CAN/FDF/BRS关闭。帧合同为：

| ID | 内容 | 频率 |
|---|---|---:|
| `0x080` | 心跳、状态、uptime | 20 Hz |
| `0x100` | 加速度，milli-m/s² | 200 Hz |
| `0x101` | 角速度，milli-rad/s | 200 Hz |
| `0x102` | 左右轮/车速，milli-m/s | 100 Hz |
| `0x103` | 姿态，milli-rad | 200 Hz |

每帧固定8字节、小端：D0:D1为独立16位序号；三轴/三速度帧的D2:D7为三个有符号16位定点值；心跳D2:D3为状态、D4:D7为ms uptime。缩放和完整字节合同见`docs/decisions/ADR-005-three-node-can-and-usb-supervision.md`。

每类流独立检查重复、乱序、gap和重启；CAN硬件CRC负责单帧链路完整性，应用层用ID/IDE/RTR/DLC、序号和age做质量门。IMU/轮速/心跳stale阈值分别为20/30/100 ms。轮周长未实测，`0x102`当前安全输出0；硬件急停未映射，心跳始终置`ESTOP_ACTIVE`且不置`CHASSIS_READY`。MSPM0没有任何电机或底盘命令API。

当前MSPM0总线调度为720帧/s，11位Classic CAN最坏位填充估算约占1 Mbps的10%；即使未来RS00扩展反馈达到500 Hz，总线预计仍低于20%。树莓派64字节视觉帧不走CAN，继续通过USB发送，避免分片、重组和不确定仲裁延迟。

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

- 初次健康对拍：`msp_rx=437108`、`invalid=0`，heartbeat/accel/gyro/attitude/wheel五类valid均为1。
- 错接压力曾触发MSPM0 bus-off：`ECR=0xF8`、`PSR=0x7E7`、TX pending=1。新增恢复逻辑先等待1 s，持续故障最多每秒尝试一次，并在健康后重武装；没有为测试而再次故意短接实车总线。
- 主机测试：`4 passed`，覆盖协议、集成、恢复首延迟/限频/重武装/32位ms回绕；ARM GCC完整链接通过。
- 本机`build-keil.bat`因未找到Keil ArmClang在工具探测阶段退出；实际镜像改用已安装的Arm GNU 9.2.1、MSPM0 SDK 2.05.01.00和相同生成配置完整重编译。这是本机工具路径差异，不是代码构建失败。
- 200 Hz姿态ELF：`D:\2026_TI\.codex_tmp\mspm0_gcc\build_200hz_os\wit-oled-hardware-spi.elf`。
- 200 Hz姿态HEX：`D:\2026_TI\.codex_tmp\mspm0_gcc\build_200hz_os\wit-oled-hardware-spi.hex`。
- SHA-256：ELF `CDB39EDCF39835A6BA35158BBA429DFA7586EAEBC040E40CAF340DA26892AB93`；HEX `D9AABCE54CCA0C215533CDF7F0C1C6BEB7368C6D15533985398B3231C7A50672`。
- TI官方CMSIS pack：`TexasInstruments.MSPM0G1X0X_G3X0X_DFP.1.3.1.pack`，SHA-256 `071BD317FC0F152DED6B2AE594D79C6FC5EB9952370526B4C14EF5B3B9807860`，只存于`.codex_tmp/`，不提交Git。
- Horco CMSIS-DAP烧录明确报告擦除/编程`46,080 bytes / 45 pages`，使用`resume_on_disconnect=true`，烧录后目标运行。
- 14 s只读日志：`can_rx`从`1,486,327`增到`1,495,794`，按实际到达时间约`729.1 frame/s`，与720帧/s目标及板间时钟误差一致；所有样本`tx_fail=0`。
- 实机测试保持电机/底盘动力断开、车轮悬空无载、调试器/USB供电、人工拔线接管；未发送enable、set-zero、位置、速度或力矩命令。

### 软件回归

- `python -m pytest firmware/edgetalk/tests vision/raspberrypi/tests -q` -> `51 passed`。
- `python -m pytest firmware/mspm0/wit-oled-hardware-spi/tests -q` -> `4 passed`。
- `python -m pytest experiments/h_ball_control_sim/tests -q` -> `47 passed`。
- 集成M33 SCons/ARM链接通过；树莓派USB守护测试为17项，包含by-id选择、独占、重连、PING-only和systemd安全合同。

## M55状态

官方FreeRTOS CM55临时工程已完成180/180源文件编译和单核链接，但仍不能烧录：临时工程`.hball_ipc_shared=0x262FC000`，当前M33为`0x261C0000`，两核地址不一致，且缺少匹配的多核后处理包。M33日志中的IPC读取失败/无shadow符合预期；在共享地址统一前禁止烧录M55或接入执行器。

## 下一步：从遥测打通到可控闭环

1. 记录IMU型号/固件、CAN收发器型号和EN/STB接法、轮周长与编码器counts/rev，再把`0x102`从0改成标定米制速度。
2. 查RS00厂家资料或做只读抓包，冻结实际角度/速度反馈帧和250~500 Hz反馈周期；在此之前不发送模式切换、使能或位置命令。
3. 用逻辑分析仪测720帧/s下的真实总线占用、MSPM0 SysTick预算和WIT UART丢帧；若ISR预算不足，把浮点/64位换算移到主循环，并限制单次FIFO排空预算。
4. 树莓派接入真实120 Hz ROI圆心数据，验收64字节USB帧的P95延迟、最长空窗、CRC/序号和自动重连；5帧拟合速度只作为Kalman观测。
5. 统一M33/M55的`.hball_ipc_shared`地址并生成匹配多核包，只先运行`SHADOW_ONLY`；禁止在IPC一致前烧录当前临时M55包。
6. 运动前按`LQR_MODEL.md`先完成空载/带球摆杆阶跃辨识，把时常、延迟、死区、回差和增益回填仿真。未来新增TX入口时在最底层加入命令ID白名单、状态机、限位和硬件急停，不得只依赖`MOTOR_COMMAND_TX=0`字符串。

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

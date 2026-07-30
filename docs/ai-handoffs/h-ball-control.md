# H题车载滚球控制交接

AI identity: Codex

Role: H-ball EdgeTalk USB/CAN/LQG integration + RS00两连杆仿真

Updated: 2026-07-30

## 当前结论

后续实现与审计的第一入口是
[`docs/architecture/h-ball-control-implementation-plan.md`](../architecture/h-ball-control-implementation-plan.md)。
该文档区分当前已打通链路、shadow代码、目标OOSM KF+LQI和每阶段验收门。

正式控制链仍为“天猛星MSPM0G3507 + 树莓派 + EdgeTalk M33/M55 + RS00”。F407不进入；
NanoPi-M5仅作树莓派性能或接口不达标时的单机备选；`PSOC_E84_robot`只提供工具链和
RS00协议参考，不迁移机械臂业务、零点或运动参数。

本轮已经完成三条真实链路：

1. 树莓派与EdgeTalk USB CDC已部署开机守护，并通过真实重启、EdgeTalk重刷断连和自动重连验收。
2. EdgeTalk与5号RS00在`1 Mbps Classic CAN`下完成一次人工只读Get_ID，TX/ACK/回复正常，所有CAN错误计数为0。
3. 更正接线后，MSPM0G3507以11位标准帧持续发送五类遥测，EdgeTalk在同一总线上同时接收RS00扩展帧；五类解析有效，200 Hz姿态镜像实测总接收约729.1帧/s。该数字是CAN镜像率，不是IMU源采样率。

因此可以宣称“三节点CAN物理链路、标准/扩展帧共存和MSPM0遥测合同已打通”。不能宣称
滚球运动闭环已完成：RS00目前只验证了只读Get_ID，连续250~500 Hz角度反馈、真实100 Hz
视觉、M33/M55共享地址统一、两连杆逆解和执行器控制仍待完成。

当前分支为`prep/2026`，未创建或填充`main`。所有自动任务保持`MOTOR_COMMAND_TX=0`、
`ACTUATOR_TX=0`，M55只允许`SHADOW_ONLY`。实机只发送过人工触发的一次RS00只读Get_ID；
从未发送enable、set-zero、位置、速度或力矩。

## 当前数据路径

```text
树莓派100 Hz灰度ROI/轮廓/圆心
  -> 64 B BALL_MEASUREMENT_V1
  -> M33 emUSB流解析/CRC/序号/age

MSPM0五类标准帧 + RS00扩展帧（当前仅Get_ID；连续角度反馈待启用）
  -> 1 Mbps Classic CAN
  -> M33质量门/200 Hz sensor_snapshot
  -> shared SRAM IPC
  -> M55 FreeRTOS 200 Hz观测器/LQG（SHADOW_ONLY，尚不可部署）
  -> M33 1 kHz安全监督
  -> ACTUATOR_TX=0
```

M33是USB、CAN、输入有效性和最终安全门的唯一所有者；M55不链接CAN或执行器发送。
LVGL数据钩子为10 Hz，只显示本题参数和`SHADOW / TX OFF`。

## 已实现算法与目标算法必须区分

当前M55源码运行的是较早Python模型的200 Hz LQG shadow：

- `ax/ay/pitch/gz`进入车体运动扰动前馈；
- 球位置和估计球速进入外层LQG；
- RS00实际角度进入执行器状态和跟随误差；
- 轮速用于诊断/前馈校验，不增加三状态LQR维数。

它没有执行器发送，也尚未包含当前两连杆逆解。新Simulink基线推荐的部署目标是：

```text
100 Hz带曝光时间戳的球位置
  + 200 Hz MSPM0 CAN惯性/姿态遥测
  -> 200 Hz历史回溯OOSM Kalman，状态[x, v, d]
  -> LQI + IMU前馈 + 低带宽扰动补偿
  -> 端部预测保护 + 抗积分饱和
  -> 水管角限幅/限速
  -> 连续装配分支的两连杆逆解
  -> M33安全门
  -> RS00 CSP目标（当前TX关闭）
```

不要把LQG shadow写成已经部署的OOSM LQI，也不要同时叠加高带宽LADRC/ESO和卡尔曼扰动
状态。ADRC保留作离线对照；MPC优先用于地图速度/加速度规划，不作为首版滚球内环。

## MSPM0 CAN遥测合同

MSPM0只新增CAN相关接口与构建接入：PA26=`CANFD0_CANTX`、PA27=`CANFD0_CANRX`，
40 MHz MCAN时钟，1 Mbps、87.5%采样点，Classic CAN/FDF/BRS关闭。

| ID | 内容 | 频率 |
|---|---|---:|
| `0x080` | 心跳、状态、uptime | 20 Hz |
| `0x100` | 加速度，milli-m/s² | 200 Hz |
| `0x101` | 角速度，milli-rad/s | 200 Hz |
| `0x102` | 左右轮/车速，milli-m/s | 100 Hz |
| `0x103` | 姿态，milli-rad | 200 Hz |

每帧固定8字节、小端。缩放和完整字节合同见
`docs/decisions/ADR-005-three-node-can-and-usb-supervision.md`。每类流独立检查重复、乱序、
gap和重启；重复或乱序帧不能刷新freshness。IMU/轮速/心跳stale阈值分别为20/30/100 ms。
轮周长未实测，`0x102`当前安全输出0；硬件急停未映射，心跳始终置`ESTOP_ACTIVE`且不置
`CHASSIS_READY`。MSPM0没有任何电机或底盘命令API。

当前MSPM0总线调度为720帧/s，11位Classic CAN最坏位填充估算约占1 Mbps的10%；即使未来
RS00扩展反馈达到500 Hz，总线预计仍低于20%。64字节视觉帧继续通过USB发送，不走CAN。

2026-07-30复审确认WIT UART当前仅`9600 bit/s`，三类11字节帧理论上最多约29.1个
完整组/s。MSPM0已改用各类WIT实际接收计数作`0x100/0x101/0x103`源序号，重复的
200 Hz CAN镜像不再伪装成新样本。统一epoch和源时刻仍需按
`shared/protocol/MSPM0_CAN_TELEMETRY_V2_PROPOSAL.md`实现。

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
- 用户systemd服务`hball-edgetalk-usb.service`为`enabled`、`active`，linger=`yes`。
- 树莓派真实重启后服务自动启动并收到READY；EdgeTalk重刷时服务能自动重连。
- 守护只发换行和PING，不发合成有效球位置，不访问CAN。

### MSPM0

- 初次健康对拍：`msp_rx=437108`、`invalid=0`，heartbeat/accel/gyro/attitude/wheel五类valid均为1。
- 错接压力曾触发bus-off；恢复逻辑先等待1 s，持续故障最多每秒尝试一次，并在健康后重武装。
- 主机测试覆盖协议、集成、恢复首延迟/限频/重武装/32位ms回绕；ARM GCC完整链接通过。
- 14 s只读日志实测约`729.1 frame/s`，与720帧/s目标及板间时钟误差一致，`tx_fail=0`。
- 实机测试保持电机/底盘动力断开、车轮悬空无载；未发送任何运动命令。

### 软件回归

- `python -m pytest firmware/edgetalk/tests vision/raspberrypi/tests -q`：当前54项通过。
- `python -m pytest firmware/mspm0/wit-oled-hardware-spi/tests -q`：当前4项通过。
- `python -m pytest experiments/h_ball_control_sim/tests -q`：旧模型29项回归通过。
- MSPM0 ArmClang全量链接通过，`Code=38324 RO-data=15104 RW-data=144 ZI-data=5968`；
  20个警告均来自既有OLED字库的超长字符串初始化。
- 集成M33 SCons/ARM链接通过；当前自动测试均不使能执行器。

## M55状态

官方FreeRTOS CM55临时工程已完成180/180源文件编译和单核链接，但仍不能烧录：
临时工程`.hball_ipc_shared=0x262FC000`，当前M33为`0x261C0000`，两核地址不一致，
且缺少匹配的多核后处理包。M33日志中的IPC读取失败/无shadow符合预期；在共享地址统一前
禁止烧录M55或接入执行器。

## RS00两连杆与100 Hz Simulink基线

已确认尺寸和当前建议值：

| 项目 | 数值 | 性质 |
|---|---:|---|
| 水管/摇杆有效长度`CB` | 250 mm | 用户给定 |
| RS00主动曲柄`OA` | 51.5 mm | 用户给定 |
| 蓝色连接杆`AB` | 65.5 mm | 用户给定 |
| RS00轴心`O`高度 | 38 mm | 用户给定 |
| 支撑台最低高度 | 50 mm | 用户最终更正 |
| 推荐合页轴心`C`高度 | 82 mm | 当前几何设计值，建议±3 mm可调 |
| 水平时`B-O`水平偏置 | 约18 mm | 当前几何设计值 |

82 mm是合页轴心高度，不是台面高度；若外径约50 mm的水管由台面直接托住，台面约57 mm。
当前几何连续可达区约`-10.3°`到`+16.9°`，正常优先`±5°`、恢复约`±7°`、硬限位暂定
`±8°`。在`±8°`内最小`|dtheta/dq|=0.3981`，最大速度/力矩换算约`2.51`。

MATLAB/Simulink R2025b于2026-07-30重跑：

- 100 Hz正常车辆场景：RMS `1.750 mm`、峰值`5.000 mm`，全程在`±10 mm`。
- 100 Hz启动/制动/坑洼：RMS `2.721 mm`、峰值`6.821 mm`，全程在`±10 mm`。
- 从`+25 mm`开始的最坏组合恢复：优化方案RMS `10.48 mm`，仅KF-LQI为`13.17 mm`；
  两者未掉球，但这不是“全程1 cm”测试。
- 人为`3.0 m/s²`轴向超限脉冲：峰值`71.99 mm`、滑移`272.92 mm/s`，机械安全失败，
  用于标记物理能力边界，不是小车`3 m/s`速度。

100、60、30 Hz在两组固定随机对照中都严格通过1 cm；100 Hz不在每个单次随机指标上都最小，
原因包括噪声/丢帧相位、整帧延迟量化和滤波器未按各频率重整定。实机必须使用曝光时间戳
做OOSM历史更新，不能照搬Simulink约40 ms整帧延迟近似。仿真是中等保真度工程模型，不是
数字孪生；摩擦、RS00响应、视觉延迟和道路谱均需实测辨识。

入口文件：

- `experiments/h_ball_control_simulink/README.md`
- `experiments/h_ball_control_simulink/钢球水管平衡系统算法与Simulink仿真说明.docx`
- `docs/decisions/ADR-006-rs00-two-link-deployment-baseline.md`
- `docs/hardware/measured-parameters.md`

## 下一步：从遥测打通到可控闭环

1. 记录IMU型号/固件、支持的波特率/输出率、CAN收发器型号和EN/STB接法、轮周长与编码器counts/rev，再把`0x102`从0改成标定米制速度。
2. 查RS00厂家资料或做只读抓包，冻结实际角度/速度反馈帧和250~500 Hz反馈周期；在此之前不发送模式切换、使能或位置命令。
3. 用逻辑分析仪测720帧/s下的真实总线占用、MSPM0 SysTick预算和WIT UART丢帧；若ISR预算不足，把浮点/64位换算移到主循环。
4. 树莓派接入真实100 Hz ROI圆心数据，验收64字节USB帧的P95延迟、最长空窗、CRC/序号和自动重连。
5. 统一M33/M55的`.hball_ipc_shared`地址并生成匹配多核包，只先运行`SHADOW_ONLY`。
6. 实测机构孔距、RS00水平零位、装配分支、摩擦和小角度阶跃，把参数回填Simulink。
7. 以固定输入向量逐帧对拍OOSM KF、LQI、端部保护和两连杆逆解，再接入M55 shadow。
8. 新增TX入口时在最底层加入命令ID白名单、状态机、限位和硬件急停；用户人工确认前保持`ACTUATOR_TX=0`。

## 工作树保护

不得擅自删除用户未跟踪的实验、配置和临时产物。任何自动测试都不得解锁RS00、发送运动目标
或发起车辆自主运动。

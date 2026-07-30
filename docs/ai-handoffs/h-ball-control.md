# H题车载滚球控制交接

AI identity: Codex

Role: H-ball EdgeTalk USB/CAN/LQG integration + RS00两连杆仿真

Updated: 2026-07-31

## 当前结论

后续实现与审计的第一入口是
[`docs/architecture/h-ball-control-implementation-plan.md`](../architecture/h-ball-control-implementation-plan.md)。
该文档区分当前已打通链路、shadow代码、目标OOSM KF+LQI和每阶段验收门。

正式控制链仍为“天猛星MSPM0G3507 + 树莓派 + EdgeTalk M33/M55 + RS00”。F407不进入；
NanoPi-M5仅作树莓派性能或接口不达标时的单机备选；`PSOC_E84_robot`只提供工具链和
RS00协议参考，不迁移机械臂业务、零点或运动参数。

本轮已经完成四端基础链路和一次受限电机微动：

1. 树莓派与EdgeTalk USB CDC已部署开机守护，并通过真实重启、EdgeTalk重刷断连和自动重连验收。
2. EdgeTalk与5号RS00在`1 Mbps Classic CAN`下完成六项参数读回、CSP模式设置、enable、`+10 mrad`、回位和stop，所有CAN错误计数为0。
3. 更正接线后，MSPM0G3507以11位标准帧持续发送五类遥测，EdgeTalk在同一总线上同时接收RS00扩展帧；五类解析有效，200 Hz姿态镜像实测总接收约729.1帧/s。该数字是CAN镜像率，不是IMU源采样率。

4. 同一集成镜像上，树莓派USB守护保持`enabled/active`，EdgeTalk端`109/109` PING/PONG零失败；MSPM0约778 Hz遥测与RS00人工控制并存。
5. JY901S UART为115200 bit/s，MSPM0上电按官方顺序设`RSW=0x000E`。10 s实测
   `0x51/0x52/0x53`各`+1996`（约199.6 Hz），checksum/unknown增量为0；EdgeTalk
   心跳`status=0x0003`、统一快照`imu_age=4 ms`。

因此可以宣称“树莓派USB、MSPM0 CAN、EdgeTalk M33和RS00人工执行器基础闭环已打通”。不能宣称滚球运动闭环已完成：真实100 Hz视觉、M33/M55共享地址统一、两连杆逆解和M55算法到正式200 Hz CSP发布仍待完成。

当前分支为`prep/2026`，未创建或填充`main`。所有自动任务保持`ACTUATOR_TX=0`，M55只允许`SHADOW_ONLY`。M33另有开机禁用、FinSH口令触发的人工CSP台架路径；已经发送enable和有界位置目标，但从未发送set-zero、速度、Iq或力矩命令。

## 2026-07-31变更、验证与继续入口

本次主要变更文件：

- `firmware/edgetalk/include/hball_rs00_control.h`与`src/hball_rs00_control.c`：CSP白名单编码和纯C台架状态机。
- `firmware/edgetalk/rtthread/hball_bench_app.c`：FinSH人工命令、单worker发送、100 Hz定向读回、回位/超时stop。
- `shared/protocol/RS00_CSP_BENCH_V1.md`：精确帧合同和实测结果。
- `docs/decisions/ADR-007-rs00-manual-csp-commissioning.md`：人工验收层与正式M55控制边界。
- `vision/raspberrypi/systemd/hball-edgetalk-camera-user.service`：真实相机桥接的无身份硬编码用户服务，替换PING-only守护。

验证为58项EdgeTalk/树莓派主机测试通过，GCC 13.3 ARM链接`text=133552/data=2152/bss=256449`，SMIF raw/XIP校验通过，以及实物`1.684 -> 1.694 -> 1.687 rad -> stop`。真实相机桥接5.8 s收到682帧，约117.6 Hz；并发微动时视觉位置有效、MSPM0约778.7 frame/s、RS00到达目标，CAN/USB均零失败。当前风险是H题机构零点/方向/传动比/机械限位未标定，M33/M55共享地址仍不一致，视觉置信度还是临时常数。下一步先做`+-10/20 mrad`阶跃辨识，再冻结正式200 Hz M33 CSP发布器；不要直接把M55 shadow目标接入人工台架API。

## 当前数据路径

```text
树莓派100 Hz灰度ROI/轮廓/圆心
  -> 64 B BALL_MEASUREMENT_V1
  -> M33 emUSB流解析/CRC/序号/age

MSPM0五类标准帧 + RS00扩展反馈/参数帧
  -> 1 Mbps Classic CAN
  -> M33质量门/200 Hz sensor_snapshot
  -> shared SRAM IPC
  -> M55 FreeRTOS 200 Hz观测器/LQG（SHADOW_ONLY，尚不可部署）
  -> M33 1 kHz安全监督
  -> ACTUATOR_TX=0（正式算法）

人工台架例外：M33 FinSH口令 -> CSP白名单 -> 100 Hz mechPos/mechVel验证 -> stop
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
  -> RS00 CSP目标（正式算法TX关闭；人工10 mrad台架路径已验证）
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

JY901S `UART_WIT`为`115200 bit/s`，三类源帧均实测约199.6 Hz。MSPM0使用
各类实际接收计数作`0x100/0x101/0x103`源序号，重复CAN镜像不伪装成
新样本。统一epoch和源时刻仍需按
`shared/protocol/MSPM0_CAN_TELEMETRY_V2_PROPOSAL.md`实现。

## 实机与构建证据

### EdgeTalk M33

- 构建模式：`HBALL_USB_ONLY=0`、`HBALL_INTEGRATED_SHADOW=1`。
- 运行标签：`0.5.0-m33-manual-small-step`。
- ELF：`text=133552 data=2152 bss=256449`。
- SHA-256：ELF `AA11BE8E23D8148260EB29139EFD2C87BD36DFC6EED40973D96CE4C62E625889`；NS HEX `7DD42E806F62336AF201D54FD3C72BEBC5AEF9E5F18E4FAEA152D3782B0D3456`；raw Secure+NS `FFD47ABD4F13459E67E9A3F5EFB4867CDBF6FD7F581812227018B23DE4401A8A`；XIP verify `9E8850EE2315118FA3628A42E352883BCF0BAA90A41C68C958077F1F7B4C614D`。
- OpenOCD预检确认`cat1d.cm33.smif1_ns`位于`0x60000000–0x67FFFFFF`。
- 写入/校验：raw `241,664 bytes`、组合XIP `237,004 bytes`，随后到达Non-secure reset handler。
- 运行日志持续输出200 Hz快照、1 kHz安全监督和`ACTUATOR_TX=0`；人工台架路径单独计数。

### RS00 CAN

- 六项只读参数`run_mode/mechPos/Iq/mechVel/VBUS/rotation`已收齐，`valid=0x3f`。
- 人工台架：`1.684 rad -> 1.694 rad -> 1.687 rad -> stop`，限速0.5 rad/s、限流0.8 A、fault=`0x00`。
- `tx=769/769/0`、TEC/REC=0、bus-off=0、FIFO full/lost=0；100 Hz定向`mechPos/mechVel`证明目标真实落地。
- 该结果证明人工执行器路径，不代表M55正式200 Hz控制发布已经启用。

### 树莓派 USB

- EdgeTalk枚举为`058b:0282`、High-Speed `cdc_acm`，稳定by-id路径存在。
- 用户systemd服务`hball-edgetalk-usb.service`为`enabled`、`active`，linger=`yes`。
- 树莓派真实重启后服务自动启动并收到READY；EdgeTalk重刷时服务能自动重连。
- 守护只发换行和PING，不发合成有效球位置，不访问CAN。
- 本次电机镜像重刷后EdgeTalk端实测`ping_rx=109/pong_tx=109`，`tx_fail=0/rx_fail=0`。

### 树莓派真实相机

- 2026-07-31模板匹配/投影标定版本部署后，从`192.168.3.33:8080/data`静置只读采样
  15.026 s，得到1739帧；按`capture_time_us`去重仍为1739帧，有效率115.731 Hz，
  `found=true`为1739/1739。
- 位置标准差`0.025661 cm=0.25661 mm`，相对中位数P95绝对偏差`0.56805 mm`，
  中心x标准差`0.4677 px`。图像为640x480，处理耗时中位1.380 ms、P95 1.442 ms、
  最大1.591 ms；当前Simulink保守使用1.5 mm视觉噪声标准差。
- 当前`found`、固定10 px半径、固定面积和临时`confidence=0.85`不能作为正式质量门；
  还需标尺小位移、遮挡/反光误检和端到端曝光时间戳测试。
- 实机元数据ROI约`[59,199,538,60]`，已与当前仓库四点透视区域对齐。该只读检查没有发送
  CAN或运动命令。

### MSPM0

- 初次健康对拍：`msp_rx=437108`、`invalid=0`，heartbeat/accel/gyro/attitude/wheel五类valid均为1。
- 错接压力曾触发bus-off；恢复逻辑先等待1 s，持续故障最多每秒尝试一次，并在健康后重武装。
- 主机测试覆盖协议、集成、恢复首延迟/限频/重武装/32位ms回绕；ARM GCC完整链接通过。
- 本次与RS00微动并行时总线实测约`778.7 frame/s`，五类数据valid、invalid/dup/ooo/gap均为0。
- JY901S启用固件通过Horco CMSIS-DAP写入`94,208 bytes`；10 s三类源帧各`+1996`，
  校验/未知帧增量为0，静置Z轴一帧约`9.915 m/s²`。
- 实机测试保持摆杆/车轮架空无载，RS00单独完成10 mrad微动；MSPM0仍没有运动命令API。

### 软件回归

- `python -m pytest firmware/edgetalk/tests vision/raspberrypi/tests -q`：当前62项通过。
- `python -m pytest firmware/mspm0/wit-oled-hardware-spi/tests -q`：当前10项通过。
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
| 水管物理长度 | 250 mm | 用户给定；与CB和球心行程不同 |
| 视觉零点到两侧物理挡边 | 112 mm / 112 mm | 用户确认；挡边间距224 mm |
| 视觉零点到C轴 | 155 mm | 向C运动为负位置 |
| 水管摇杆`CB` | 300.1 mm | 用户最终确认 |
| RS00主动曲柄`OA` | 35.0 mm | 用户给定 |
| 蓝色连接杆`AB` | 55.5 mm | 用户给定 |
| RS00轴心`O`高度 | 38 mm | 用户给定 |
| 右侧合页轴心`C`高度 | 93 mm | 用户给定 |
| `O-C`水平距离 | 285 mm | O在C左侧 |
| 支撑台最低高度 | 50 mm | 用户最终更正 |
| 水管内半径 / 内径 | 6.5 mm / 13 mm | 圆管沿轴向切半形成半圆槽 |
| 钢球直径 / 质量 | 10 mm / 4.11 g | 钢球位于半圆槽内 |
| 滚动阻力系数 | 约0.005 | 用户给出约值，待自由滚动日志复核 |
| 水平时`B-O`水平偏置 | -15.1 mm | B在O左侧 |

视觉轴向坐标冻结为：`x=0`是树莓派检测零点，向合页C为负，远离C为正，因此
`x_C=-155 mm`、球心到C的沿管力臂为`s_C=155 mm+x`。物理挡边为`x=±112 mm`；
球心软件几何极限必须再扣除钢球半径。用户确认钢球直径10 mm、质量4.11 g，因此部署算法按
半径5 mm、球心几何极限±107 mm。水管内半径6.5 mm，理想截面径向差为1.5 mm；
轴向纯滚动的一阶方程不因此改变，但横向晃动、接触和脱槽风险必须靠实物压力测试确认。

C是右侧固定合页，水管水平时`C->B`向左；O位于C左侧285 mm、低55 mm。若外径约
50 mm且C轴与水管中心轴重合，管底约68 mm。当前雅可比保护下连续可用区约
`-6.47°`到`+6.79°`，正常优先`±4°`、恢复`±5.5°`、硬指令限位`±6°`。
`±6°`内RS00相对水平零位约需`-57.00°`到`+70.68°`，最小
`|dtheta/dq|=0.0467`，最差速度/力矩换算约`21.43`，水管指令斜率改为`0.35 rad/s`。

车身IMU仍应保留：它补偿车辆俯仰和轴向加速度，不测水管相对车身角度。相对水管角必须
由RS00编码器经过四杆正解得到；世界系水管角为
`theta_linkage(q)+pitch_vehicle`。车身IMU无法发现连杆回差和支架变形，必要时在C轴
增加直接角度编码器。JY901S的加速度、角速度和姿态源帧均已实测约199.6 Hz；
CAN重复镜像不刷新源age。

MATLAB/Simulink R2025b按新机构和115200 bit/s、200 Hz唯一IMU基线重跑：

- 100 Hz正常车辆场景：RMS `2.344 mm`、峰值`5.216 mm`，全程在`±10 mm`。
- 100 Hz启动/制动/坑洼：RMS `21.453 mm`、峰值`60.559 mm`，严格1 cm失败。
- 从`+25 mm`开始的可行组合恢复：优化方案RMS `10.618 mm`、峰值`25.000 mm`，
  仅KF-LQI为`13.462/26.010 mm`；两者机械安全，但这不是“全程1 cm”测试。
- 超新机构包线组合峰值`94.311 mm`、滑移`148.278 mm/s`，机械安全失败。
- 人为`3.0 m/s²`轴向超限脉冲：峰值`115.003 mm`、滑移`100.310 mm/s`，机械安全失败，
  用于标记物理能力边界，不是小车`3 m/s`速度。

100、60、30 Hz在正常车辆对照中都严格通过1 cm，在启动/制动/坑洼对照中都失败；
100 Hz不在每个单次随机指标上都最小，原因包括噪声/丢帧相位、整帧延迟量化和滤波器未按
各频率重整定。实机必须使用曝光时间戳
做OOSM历史更新，不能照搬Simulink约40 ms整帧延迟近似。仿真是中等保真度工程模型，不是
数字孪生；摩擦、RS00响应、视觉延迟和道路谱均需实测辨识。

入口文件：

- `experiments/h_ball_control_simulink/README.md`
- `experiments/h_ball_control_simulink/钢球水管平衡系统算法与Simulink仿真说明.docx`
- `docs/decisions/ADR-006-rs00-two-link-deployment-baseline.md`
- `docs/hardware/measured-parameters.md`

## 下一步：从遥测打通到可控闭环

### 2026-07-31 固件实现状态

- M55控制管线已切换到部署版三状态延迟KF（`x/v/等效扰动`）、LQI、车身IMU前馈、
  积分抗饱和和端部恢复；保存32个200 Hz状态，按视觉曝光时间做最长150 ms历史更新并回放。
- 四杆正逆解已使用`OA=35 mm`、`AB=55.5 mm`、`CB=300.1 mm`和
  `O-C=(-285,-55) mm`；水平几何解为`3.051858578444 rad`。
- 正常/恢复/硬角度限制为`±4°/±5.5°/±6°`，管角指令斜率为`0.35 rad/s`。
- 水平位实际RS00编码器值尚未提供，`HBALL_LINKAGE_LEVEL_ENCODER_VALID=0`。
  因此即使M55影子算法输出正常，M33安全资格也必定为false。
- M33新增50 Hz只读控制日志邮箱，USB线程发送80字节CRC32C帧；控制线程不做USB阻塞写。
  树莓派桥接程序用`--telemetry-log PATH`保存通过CRC校验的帧，原始量约4 kB/s。
  协议见`docs/protocol/EDGETALK_CONTROL_LOG_V1.md`。
- 主机测试62项通过；尚未完成目标板编译、M55烧录或实车闭环验证，正式自动运动仍锁死
  `ACTUATOR_TX=0`。

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

## 2026-07-31 现场演示任务编排与首个shadow切片

- 基于`fd059c7`完成架构规划，并已实现任务CAN V1、MSP任务客户端和M33任务仲裁器；
  对应提交为`ba707fd`、`ded48ca`和`95cc986`。
- 冻结方向为：MSPM0上的SW3/SW1是唯一操作员入口，M33是唯一分布式任务仲裁器，
  M55只执行带任务上下文的滚球算法，树莓派常驻视觉/录像/回放。
- 赛题第1项不是SW3选项；SW3只循环官方Q2到Q6。每次有效SW1使用一个
  `mission_epoch`关联CAN、IPC、USB日志、视频和结果；Q6只在START时锁存一次初始球位。
- MSP与M33之间已有独立任务控制面；双核任务IPC和USB任务marker仍未实现，不能让M55或
  树莓派根据本地事件自行猜测START。
- NanoPi-M5只作为冷备视觉主机，不进入比赛主链路。
- 规划文档为：
  - `docs/decisions/ADR-008-competition-demo-mission-orchestration.md`；
  - `docs/architecture/competition-demo-state-machine.md`；
  - `docs/architecture/system-overview.md`；
  - 本交接文档。
- 验证结果：EdgeTalk与MSP主机测试合计`60 passed`；MSP Keil固件构建成功；M33集成工程
  构建成功并生成重定位后的`build/rtthread.hex`。树莓派`192.168.3.33`已可达，Windows可见
  EdgeTalk `COM26`及USB串行口`COM11`。尚未烧录本次固件，也没有发送运动命令。
- 主要风险仍是裁判计时口径、100/120 Hz正式视觉门限、A/B地标判据、硬急停映射和
  RS00正式反馈/限位尚未冻结；这些项目已经列入规划文档的实施前参数表。
- 决策见`docs/decisions/ADR-008-competition-demo-mission-orchestration.md`；完整状态、READY
  掩码、协议草案、实施切片和现场SOP见
  `docs/architecture/competition-demo-state-machine.md`。
- 下一步在车轮/动力/急停/操作者条件明确后烧录MSP与M33，只验证`0x081/0x084`上行和
  `0x082`下行、epoch/Q号/状态序号一致，预期停在PREPARING且`ACTUATOR_TX=0`。随后再把
  MSP现有SW3/SW1菜单接入任务client和缺失READY显示。不得跳过Checkpoint A/B直接开放
  正式运动；SW1长按中止语义仍待负责人确认。

## 2026-07-31 任务CAN与四端数据现场验证

- 安全条件由操作者现场确认：车轮架空或底盘动力断开，硬急停为直接断电，操作者在旁可
  立即接管。本轮没有发送RS00 enable、位置、速度、电流或车辆运动命令。
- EdgeTalk使用Infineon OpenOCD 2.0.0和KitProg3烧录。写入前确认
  `cat1d.cm33.smif1_ns`位于`0x60000000`；合并镜像实际写入241664 bytes、verify
  238476 bytes并返回`Verified OK`。复位后M33打印CAN 1 Mbps初始化成功和
  `ACTUATOR_TX=0`。
- MSPM0G3507使用pyOCD 0.44.1、TI DFP 1.3.1和Horco CMSIS-DAP烧录，擦除并编程
  53248 bytes。`pyocd load`完成后旧程序仍可能继续执行，必须再执行一次显式
  `pyocd reset`，并以新协议计数而不是烧录退出码判断新镜像已经接管。
- 显式复位后M33收到Q2、epoch 1、PREPARE：现场快照为`intent=2314/0`、
  `chassis=5781/0`、`status_tx=2888/0`。MSP RAM现场读到有效mission status接收计数，
  证明`0x081/0x084`上行与`0x082`下行均已通过。
- 全局状态保持PREPARING：`ready=0x0067`、`required=0xffff`、`start=0/0`。当前只满足
  M33、MSP链路、IMU、Pi USB和视觉；录像ACK、M55任务IPC、正式控制、安全、RS00配置、
  起始几何和配置哈希等位仍为0，因此不会进入READY/RUNNING。
- 同一快照中树莓派视觉为114.8 Hz，累计65719帧，CRC/乱序/gap均为0；RS00六类参数
  读回有效，母线22.597 V、mode 0，人工运动层为SAFE且`manual_tx=0`。这证明Pi、MSP、
  EdgeTalk和RS00的数据汇总链路在线，不代表M55任务IPC或正式执行器闭环已完成。
- 下一步是把MSP现有SW3/SW1菜单正式映射到Q2～Q6 mission client，并显示M33返回的首个
  READY缺失原因；在此之前按键仍只控制旧本地菜单，不能用于分布式评分任务。

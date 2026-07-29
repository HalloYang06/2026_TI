# H题车载滚球控制交接

AI identity: Codex

Role: H-ball EdgeTalk USB/CAN bring-up

Updated: 2026-07-30

## 当前状态

已完成题目校核、三板控制边界、非线性前馈 + 增广 LQG 数学模型、延迟/离群视觉观测器、5 帧球速估计、摩擦与多速率仿真，以及 72 组分级压力测试。已将成果整理到目标仓库 `HalloYang06/2026_TI` 的 `prep/2026` 分支；数值模型保留在 `experiments/`，固件适配保留在 `firmware/`，所有硬件路径在重复台架验证前仍按实验原型管理。

EdgeTalk USB CDC 已新增 `HBALL_USB_ONLY=1`、P16.5 心跳和树莓派 PING/PONG 探针，并按 Infineon 官方 CDC echo 切换到 emUSB 2.1.0.3859。M33 编译、Secure+NS 合并、烧录、raw/XIP/NS 校验和 Non-secure 启动均成功，蓝灯正常闪烁。当前 FinSH 为 `state=0x11 configured=0 conn=1 cfg=0 actuator_tx=0`，即仅 `ATTACHED|SUSPENDED`；树莓派无 `lsusb` 设备、无 `/dev/ttyACM*`、无 USB reset 证据，不能记为已打通。

基线提交：

- `f9c4e02 feat(sim): add H-problem LQG stress model`
- `97c7e1e docs(arch): define H-problem three-board baseline`
- `origin/prep/2026` 已推送并设为本地上游；后续参数记录继续提交到该分支，未创建或填充 `main`。

最终候选架构：

- 天猛星 MSPM0G3507：红外循迹、底盘速度环、IMU UART DMA、200 Hz 状态发布。
- 树莓派：60 Hz 钢球视觉、时间戳/置信度、比赛视频显示与存储。
- EdgeTalk：200 Hz 球状态估计和 LQG、1 kHz 摆杆角度环、20 kHz FOC。
- F407 与 NanoPi M5 不进入正式链路。

## 算法定稿

- 状态反馈：`[位置误差, 估计球速, 摆杆角误差]`。
- 观测状态：`[球位置, 球速度, 等效扰动偏置]`。
- LQR：`Q=diag(1200,25,2)`、`R=5`、`Ts=5 ms`、标称执行器时常 `25 ms`。
- 增益：`K=[14.56338, 3.31547, 0.58093]`。
- 钢球按实心球纯滚动处理：`J=2/5*m*R^2`，等效滚动系数为 `5/7`，已包含平动和转动惯性；打滑、自转独立状态和钢球对摆杆的反作用力矩仍待实物辨识。
- 钢球质量已实测为 `4.11 g`；直径暂按题目约 `10 mm`，暂定 `J=4.11e-8 kg*m^2`，只因质量更新无需重算LQR。
- 可测扰动：纵向/横向加速度、俯仰、偏航角速度和位置相关离心项做非线性前馈。
- 视觉：60 Hz、采集时间戳延迟补偿、5 帧二次最小二乘球速、4σ 创新门限、连续拒帧协方差膨胀恢复。
- 约束：摆角 `+-4 deg`，目标角变化率 `80 deg/s`。

## 验证结果

- `python -m pytest -q experiments\h_ball_control_sim\tests`：`29 passed`。
- `python -m compileall -q experiments\h_ball_control_sim`：通过。
- `python experiments\h_ball_control_sim\run_lqr_study.py`：重新生成基准、消融和题目功能项结果。
- `python experiments\h_ball_control_sim\run_stress_campaign.py`：重新生成72组CSV和图表，结果与文档一致。
- 静止 `0 -> +5 cm -> -5 cm`：持续进入 `+-1 cm` 带的时刻为 `0.492 s` 和总时刻 `3.070 s`。
- 运动中保持任意 `+5 cm`：峰值误差 `1.60 mm`。
- 前馈消融：RMS 从 `0.883 mm` 降到 `0.371 mm`。
- 72 组压力测试：0/1/2 级均 `12/12` 通过，2 级最坏 `6.48 mm`；3 级 `10/12`，首次超过 1 cm；5 级 `0/12`。

## 关键文件

- `docs/architecture/system-overview.md`：三板闭环、频率、传感器与降级策略。
- `docs/reference/infineon-edgetalk-motor5.md`：参考仓库中EdgeTalk、5号电机、构建与烧录流程的提取记录。
- `docs/hardware/measured-parameters.md`：实测质量、暂定转动惯量、摆杆负载和下一步测量。
- `experiments/h_ball_control_sim/LQR_MODEL.md`：模型、参数、频率和实物门槛。
- `docs/decisions/ADR-001-h-ball-control-architecture.md`：三板与算法决策。
- `experiments/h_ball_control_sim/hballsim/controllers.py`：LQR、观测器和球速估计。
- `experiments/h_ball_control_sim/hballsim/simulation.py`：非线性多速率闭环。
- `experiments/h_ball_control_sim/hballsim/stress.py`：分级干扰生成器。
- `experiments/h_ball_control_sim/hballsim/campaign.py`：压力战役、CSV和图表。
- `experiments/h_ball_control_sim/output/stress_trials.csv`：每个随机种子的完整参数与结果。
- `experiments/h_ball_control_sim/output/stress_envelope.png`：最终压力包络。
- `experiments/h_ball_control_sim/output/boundary_failure.png`：第一失效等级最坏种子。

## 风险与限制

- 模型是一维近似，未完整覆盖槽内横向摆动、球自旋、PPR 管弹性、连杆间隙和车架扭转。
- 压力等级是工程故障注入，不代表现场概率；跨等级相关性不能当独立因果贡献率。
- 尚未获得 IMU 型号、摄像头/镜头、无刷编码器分辨率、摆杆传动比、EdgeTalk 固件版本和具体引脚。
- 当前 Git 分支为 `prep/2026`；不得创建、填充或合并到 `main`，直到仓库所有者明确批准稳定基线。
- `PSOC_E84_robot` 仅为参考仓库；不能复制其医疗机械臂零点、方向、限位、关节映射或安全权限结构。
- PSE84 临时构建树位于仓库 `tmp/` 且带既有启动诊断改动，不是可提交产品 BSP；可复用源保留在 `firmware/edgetalk/`。
- menuconfig 截图里的通用 `Using USB -> Using USB host/device` 对应 RT-Thread 旧 USB 栈，不是 emUSB；两项保持关闭，只开启板级 `BSP_USING_USB`。
- SCons 根目录 `rtthread.hex` 只有 NS XIP 段，禁止直接烧录；必须使用合并后的 Secure+NS raw 镜像。
- 用户已确认数据线具备数据能力；`P17.4 VBUS_DETECT` 当前读低，DWC2 `DCTL=0` 且没有主机 reset，下一台电脑应优先核对树莓派 Host 端口、EdgeTalk Device 端口和板级 VBUS 检测/供电路径。

## 下一步

用户给出具体引脚和硬件型号后：

1. 建立三板接口表和坐标系约定，明确 UART/CAN/编码器/FOC 引脚、电平和供电。
2. 先写二进制状态包定义：版本、序号、采集时间戳、IMU量、底盘量、CRC和超时策略。
3. 设计只读回放工具，用记录的视觉/IMU日志驱动 EdgeTalk 算法，禁止在自动测试中解锁电机。
4. 台架标定相机 P95 延迟/噪声、执行器时常/延迟/增益/死区、摆杆水平零点和摩擦，再回填仿真。
5. 实物测试必须车轮架空、底盘动力断开或限流、硬件急停、操作员保留断电接管。
6. USB 联调按 P16.5 心跳、FinSH `hball_usb_status`、树莓派 `dmesg/lsusb`、`/dev/ttyACM*`、`HBALL_USB_READY`、`PING/PONG` 顺序验收；任何一层失败即停线定位，不继续叠加 CAN 或算法。

## 本次 USB 续接验证

- M33：`text=190096 data=15616 bss=244300`；临时 BSP emUSB 静态契约 `6 passed`。
- raw combined SHA-256：`6FF1D97D0A833B490BD0D33FF5E615D6C66ED98522668940C49C5D13C379ACAB`。
- XIP verify SHA-256：`65EF82513764233A98BF10184F13298E61A9ED3840B2AE621A3AFD532CFA3364`。
- NS SHA-256：`453A13997A17E347EA5D5025170E247F4E35D4C79C69FEE991389E6922B2ED71`。
- 实测校验：raw `315392` bytes、XIP `308168` bytes、NS `205712` bytes，已到达 Non-secure reset handler。
- 安全状态：电机动力断开，未执行自主运动，USB 诊断永久保持 `actuator_tx=0`。

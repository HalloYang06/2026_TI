# H题车载滚球控制交接

AI identity: Codex

Role: H-ball EdgeTalk USB/CAN bring-up

Updated: 2026-07-30

## 当前状态

已完成题目校核、三板控制边界、非线性前馈 + 增广 LQG 数学模型、延迟/离群视觉观测器、5 帧球速估计、摩擦与多速率仿真，以及 72 组分级压力测试。已将成果整理到目标仓库 `HalloYang06/2026_TI` 的 `prep/2026` 分支；数值模型保留在 `experiments/`，固件适配保留在 `firmware/`，所有硬件路径在重复台架验证前仍按实验原型管理。

EdgeTalk USB CDC 已正式打通。M33 使用 emUSB 2.1.0.3859 与 `HBALL_USB_ONLY=1`；树莓派以 High-Speed 480 Mbps 枚举 `058b:0282`，`cdc_acm` 生成 `/dev/ttyACM0` 和稳定 by-id 路径。最终 FinSH 为 `state=0x1e configured=1 conn=1 cfg=1 actuator_tx=0`。双向 PING/PONG 已通过 30 秒 100 Hz 压测和 5 次关闭/重开验证，当前可进入 USB 二进制视觉协议和 CAN 只读联调。

基线提交：

- `f9c4e02 feat(sim): add H-problem LQG stress model`
- `97c7e1e docs(arch): define H-problem three-board baseline`
- `origin/prep/2026` 已推送并设为本地上游；后续参数记录继续提交到该分支，未创建或填充 `main`。

最终候选架构：

- 天猛星 MSPM0G3507：红外循迹、底盘速度环、IMU UART DMA、200 Hz 状态发布。
- 树莓派：120 Hz 灰度ROI与轮廓识别、时间戳/置信度、比赛视频显示与存储。
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
- 视觉：120 Hz目标、采集时间戳延迟补偿、5 帧二次最小二乘球速、4σ 创新门限、连续拒帧协方差膨胀恢复。
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
- emUSB OUT 必须直接阻塞调用 `USBD_CDC_Receive(..., 0)`；先查询 `USBD_CDC_GetNumBytesInBuffer()` 会导致 OUT 端点未 arm。不要用两个线程并发阻塞访问同一 emUSB 句柄。
- 树莓派探针打开 raw 串口后先发送空行做帧同步，因此 `invalid_rx` 每次打开会增加 1；这不是正式 PING 丢包。验收应看 `timeout/unexpected/tx_fail/rx_fail`。

## 下一步

用户给出具体引脚和硬件型号后：

1. 建立三板接口表和坐标系约定，明确 UART/CAN/编码器/FOC 引脚、电平和供电。
2. 先写二进制状态包定义：版本、序号、采集时间戳、IMU量、底盘量、CRC和超时策略。
3. 设计只读回放工具，用记录的视觉/IMU日志驱动 EdgeTalk 算法，禁止在自动测试中解锁电机。
4. 台架标定相机 P95 延迟/噪声、执行器时常/延迟/增益/死区、摆杆水平零点和摩擦，再回填仿真。
5. 实物测试必须车轮架空、底盘动力断开或限流、硬件急停、操作员保留断电接管。
6. USB 基础链路与64字节视觉协议已完成；下一步在120 Hz真实识别、240 Hz链路验收和200 Hz EdgeTalk消费条件下做只读延迟/丢包压测，再完成CAN实物对拍。

## 本次 USB 续接验证

- M33：`text=192804 data=14884 bss=245025`；主仓库 USB 测试 `10 passed`，临时 BSP emUSB 静态契约 `6 passed`。
- raw combined SHA-256：`D5C8FB63A28A5405088AE803A080AF483BB23B3A87A94B190ABE55B2011D1A80`。
- XIP verify SHA-256：`BFE5092E229F9D9A2D4582FB0BE118B0B05F50948154E214FADEF76FE932774B`。
- NS SHA-256：`982AC90DCA53AAF21A2E0BBC5052778440AFFDBF092956DF55D028670A8E2434`。
- 实测校验：raw `315392` bytes、XIP `310144` bytes、NS `207688` bytes，已到达 Non-secure reset handler。
- 30 秒 100 Hz：`2960/2960`、零超时、平均 RTT `1.60 ms`、P95 `2.14 ms`。
- 5 次关闭/重开：全部通过，共 `597/597`；最终可复现镜像复测 `987/987`、P95 `2.08 ms`。
- 安全状态：电机动力断开，未执行自主运动，USB 诊断永久保持 `actuator_tx=0`。

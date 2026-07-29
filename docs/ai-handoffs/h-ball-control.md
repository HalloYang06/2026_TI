# H题车载滚球控制交接

AI identity: Codex

Role: H题三板架构、LQG算法与压力仿真

Updated: 2026-07-29

## 当前状态

已完成题目校核、三板控制边界、非线性前馈 + 增广 LQG 数学模型、延迟/离群视觉观测器、5 帧球速估计、摩擦与多速率仿真，以及 72 组分级压力测试。已将成果整理到目标仓库 `HalloYang06/2026_TI` 的 `prep/2026` 分支；全部内容仍是未接硬件的实验原型，保留在 `experiments/`。

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
- 可测扰动：纵向/横向加速度、俯仰、偏航角速度和位置相关离心项做非线性前馈。
- 视觉：60 Hz、采集时间戳延迟补偿、5 帧二次最小二乘球速、4σ 创新门限、连续拒帧协方差膨胀恢复。
- 约束：摆角 `+-4 deg`，目标角变化率 `80 deg/s`。

## 验证结果

- `python -m pytest -q experiments\h_ball_control_sim\tests`：`29 passed`。
- `python -m compileall -q experiments\h_ball_control_sim`：通过。
- 静止 `0 -> +5 cm -> -5 cm`：持续进入 `+-1 cm` 带的时刻为 `0.492 s` 和总时刻 `3.070 s`。
- 运动中保持任意 `+5 cm`：峰值误差 `1.60 mm`。
- 前馈消融：RMS 从 `0.883 mm` 降到 `0.371 mm`。
- 72 组压力测试：0/1/2 级均 `12/12` 通过，2 级最坏 `6.48 mm`；3 级 `10/12`，首次超过 1 cm；5 级 `0/12`。

## 关键文件

- `docs/architecture/system-overview.md`：三板闭环、频率、传感器与降级策略。
- `docs/reference/infineon-edgetalk-motor5.md`：参考仓库中EdgeTalk、5号电机、构建与烧录流程的提取记录。
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

## 下一步

用户给出具体引脚和硬件型号后：

1. 建立三板接口表和坐标系约定，明确 UART/CAN/编码器/FOC 引脚、电平和供电。
2. 先写二进制状态包定义：版本、序号、采集时间戳、IMU量、底盘量、CRC和超时策略。
3. 设计只读回放工具，用记录的视觉/IMU日志驱动 EdgeTalk 算法，禁止在自动测试中解锁电机。
4. 台架标定相机 P95 延迟/噪声、执行器时常/延迟/增益/死区、摆杆水平零点和摩擦，再回填仿真。
5. 实物测试必须车轮架空、底盘动力断开或限流、硬件急停、操作员保留断电接管。

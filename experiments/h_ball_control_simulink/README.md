# 钢球—水管非线性 Simulink 仿真

这个模型按以下机构建立：

- 一颗实心钢球在直 PVC 水管或半剖水管内沿管轴运动；
- 水管绕固定转轴摆动；
- 灵足时代 RS00 带动主动曲柄，曲柄通过中间连杆驱动水管摇杆；
- 模型用闭环几何约束求解，不再假设电机角与水管角 1:1；
- 水管两端开放，钢球越过有效长度即判定掉球；
- 相机测量钢球在水管轴向的位置。

如果实际机构是钢球位于水管外圆顶部、二维平板，或者整根水管安装在无人机上，需要改变坐标和动力学结构，不能只改一个摩擦系数。

## 已考虑的现实因素

- 钢球平动和转动惯量；
- 球—PVC 接触面的 Stribeck 静/动摩擦过渡；
- 球相对管面的滑动速度；
- 滚动阻力矩；
- 黏性阻力和速度平方阻力；
- RS00 运控模式的位置刚度、速度阻尼和前馈力矩；
- 两连杆闭环的正/逆运动学；
- 随姿态变化的角度雅可比 `d(theta)/d(q)`；
- 雅可比产生的力矩放大、速度变换和等效惯量变化；
- 根据最不利雅可比设置的水管角速度斜率限制；
- 主动曲柄、中间连杆和水管摇杆的质量与转动惯量；
- 接近机构死点时的有效性检查；
- WIT 115200 bit/s、200 Hz唯一IMU样本、PSoC 200 Hz外环、500 Hz电机安全任务和100 Hz视觉的3 kHz公共仿真网格；
- RS00 额定/峰值力矩、速度—力矩边界、电流环延迟；
- 关节摩擦、微小回差、水管和支架转动惯量；
- 钢球位置变化对 RS00 输出轴产生的扰动力矩；
- 14 位编码器量化、CAN 指令/反馈延迟；
- RS00 热模型、75℃预警和 80℃热降额；
- 相机帧率、测量噪声、像素量化、随机丢帧和视觉延迟；
- 离散卡尔曼滤波；
- LQI/LQR 状态反馈；
- 基于位置、向外速度和制动距离的端部保护；
- 掉球判定。

## 文件

- `ball_pipe_defaults.m`：全部物理、传感器和控制参数。
- `build_ball_pipe_model.m`：自动生成 `ball_pipe_nonlinear.slx`。
- `sfun_ball_pipe_plant.m`：非线性钢球动力学。
- `sfun_rs00_joint_actuator.m`：RS00、管体负载和热模型。
- `fourbar_kinematics.m`：两连杆闭环正运动学和雅可比。
- `fourbar_inverse_kinematics.m`：水管目标角到电机目标角的逆运动学。
- `fourbar_equivalent_inertia.m`：曲柄、连杆和水管折算到电机轴的变惯量。
- `analyze_two_link_mechanism.m`：检查 ±倾角工作区、死点、传动比和等效惯量。
- `vehicle_motion_profile.m`：载车加减速、俯仰、振动和冲击工况。
- `sfun_vehicle_imu.m`：车身IMU 200 Hz唯一源样本、保持输出、噪声、偏置和处理延迟。
- `run_vehicle_stress_test.m`：低摩擦载车压力测试和算法对比。
- `run_camera_rate_comparison.m`：100、60与30 Hz视觉在1 cm误差带内的对照测试。
- `CONTROL_ARCHITECTURE.md`：树莓派、PSoC E84、MSPM0与RS00任务划分。
- `sfun_pipe_camera.m`：相机噪声、延迟和丢帧。
- `sfun_lqg_edge_controller.m`：卡尔曼 + LQI + 边缘救球。
- `run_ball_pipe_demo.m`：运行标称仿真并生成曲线。
- `run_ball_pipe_sweep.m`：比较摩擦、阻力、延迟等不确定性。
- `estimate_ball_pipe_parameters.m`：根据斜管实验估计参数。

相机默认值和输出结果均已更新为100 Hz基线。`camera_rate_comparison.png/.mat`包含
100、60与30 Hz对照，`vehicle_stress_results.mat`和压力测试图片使用100 Hz默认相机。
当前相机S-Function按整帧数近似延迟，默认35 ms在100 Hz下约等效为40 ms；实机部署应使用
曝光时间戳做历史回溯，不使用整帧延迟近似。

## 车身IMU与115200波特率

车身IMU用于测量车辆俯仰和底座轴向加速度，适合做前馈补偿；它不测水管相对车身角度。
相对水管角由RS00编码器经过四杆正解得到：

```text
theta_world = theta_linkage(rs00_q) + pitch_vehicle
```

当前WIT UART为115200 bit/s。每个完整样本组包含3个11字节8N1帧，至少330 bit，因此
串口理论上限约349.1组/s：

- 100 Hz完整组约占28.6%链路；
- 200 Hz完整组约占57.3%，是当前推荐升级目标；
- 500 Hz完整组需要约165 kbit/s，115200仍然不够。

MSPM0的`UART_WIT`已经设为115200；还要确认WIT输出率寄存器确实设为100或200 Hz。
控制器按源序号判断新样本，200 Hz CAN重复镜像不能冒充新IMU数据。默认模型使用
200 Hz唯一样本和8 ms延迟；下面的覆盖参数仅用于敏感性分析：

```matlab
vehicle_stress_selection = 5;
imu_source_rate_override_hz = 200;
imu_delay_override = 0.008;
run_vehicle_stress_test
```

## 2026-07-31实跑摘要

使用MATLAB/Simulink R2025b重跑：

| 测试 | 结果 |
|---|---|
| 标称±20 mm位置指令 | RMS 9.203 mm，未掉球；41.384 mm峰值包含瞬时正负阶跃 |
| 正常车辆，100 Hz视觉，初始误差5 mm | RMS 2.344 mm，峰值5.216 mm，严格1 cm通过 |
| 起步/制动/坑洼，100 Hz视觉，初始误差5 mm | RMS 21.453 mm，峰值60.559 mm，严格1 cm失败 |
| 可行组合恢复，优化方案，初始误差25 mm | RMS 10.62 mm，峰值25.00 mm，机械安全通过 |
| 可行组合恢复，仅KF-LQI | RMS 13.46 mm，峰值26.01 mm，机械安全通过 |
| 超机构包线组合与3.0 m/s²脉冲 | 机械安全失败，用作物理包线边界 |

相机对照测试从`+5 mm`开始，只有正常车辆工况严格`+-10 mm`通过。压力恢复测试从误差带外开始，不能把
“机械安全通过”写成“全程满足1 cm”。完整表格、算法原理、部署分工和参数辨识步骤见
`钢球水管平衡系统算法与Simulink仿真说明.docx`。

## 使用方法

需要 MATLAB、Simulink。建议 R2021b 或更新版本。

在 MATLAB 中把当前目录切换到本文件所在目录，然后运行：

```matlab
run_ball_pipe_demo
```

脚本会：

1. 生成 `ball_pipe_nonlinear.slx`；
2. 运行 14 秒闭环仿真；
3. 输出 RMS 位置误差、最大误差和掉球状态；
4. 保存 `ball_pipe_nominal_result.png`；
5. 打开生成的 Simulink 模型。

运行鲁棒性对比：

```matlab
run_ball_pipe_sweep
```

单独检查两连杆几何：

```matlab
analyze_two_link_mechanism
```

替换孔距后应先运行这个脚本。如果输出 `All requested poses valid: false`，
或者最小 `|dtheta/dq|` 接近 `bp.mechanism.minimum_jacobian`，不要直接上机，
需要调整安装孔位或减小允许倾角。

它会比较：

- 标称参数；
- 低摩擦；
- 高摩擦；
- 大视觉/执行器延迟；
- 大滚动阻力；
- 低摩擦、大延迟和高丢帧的组合情况。

## 为什么不能直接相信一个“钢—PVC 摩擦系数”

摩擦系数会受到以下因素影响：

- PVC 是硬管还是软管；
- 钢球是否镀铬、是否生锈；
- 管内是否有灰尘、水汽、油膜；
- 表面划痕和粗糙度；
- 球是否真正滚动，还是同时滚动和滑动；
- 接触载荷、运动速度和温度。

默认值只是仿真起点：

```matlab
bp.contact.mu_static  = 0.050;
bp.contact.mu_kinetic = 0.030;
bp.contact.rolling_resistance = 0.005; % 用户给出的约值
```

这是“很滑”条件的暂定包络，仍应替换成实测数值。

工程上应当做两类实验。

### 1. 判断滚动还是滑动

在钢球表面粘一个很小的可视标记，用高帧率视频同时计算球心速度 `v` 和角速度 `omega`：

```text
abs(v - radius*omega) 很小：以滚动为主
abs(v - radius*omega) 明显：存在滑动
```

### 2. 斜管下落实验

从静止释放，记录管角度、运动距离和时间。例如：

```matlab
estimate_ball_pipe_parameters(3, 0.40, 1.10, 'rolling')
```

滚动时反算等效滚动阻力；确认滑动时可改成：

```matlab
estimate_ball_pipe_parameters(3, 0.40, 1.10, 'sliding')
```

建议每个角度重复至少 10 次，使用 2°、3°、4°、5° 多个角度拟合，而不是只测一次。

## 后续需要替换的实物参数

目前已经填入：

- 水管物理长度 `250 mm`；
- 视觉零点到左右物理挡边各 `112 mm`，两挡边间距 `224 mm`；
- 视觉零点到合页C轴 `155 mm`，向C运动为负位置；
- 水管合页到蓝杆连接端 `CB = 300.1 mm`，与物理长度和挡边间距都不是同一个量；
- RS00 主动杆 `OA = 35.0 mm`；
- 蓝色连接杆 `AB = 55.5 mm`；
- RS00 输出轴高度 `38 mm`。
- 水管右侧合页轴 `C` 高度 `93 mm`；
- 固定轴 `O-C` 水平距离 `285 mm`。

当前装配关系：

- `C` 是右侧固定合页，水管水平时 `C->B` 指向左侧；
- 水管水平时，`B` 位于电机轴心 `O` 左侧约 `15.1 mm`；
- 支撑台高度满足“不小于 `50 mm`”的约束；
- 如果约50 mm外径的水管中心轴就是93 mm，管底约为68 mm；
- 当前连续安全指令区取 `±6°`，水管指令斜率取 `0.35 rad/s`。

MATLAB机构分析表明，`±6°`全部可达，RS00相对水平零位约需运动
`-57.00°`到`+70.68°`；最小`|dtheta/dq|=0.0467`，最差速度/力矩换算约
`21.43`。继续扩大到约`-6.47°/+6.79°`会进入当前雅可比死点保护区，因此旧的
`±8°/1.2 rad/s`不能沿用。

仍需实测并填写：

- 钢球直径10 mm、质量4.11 g；
- 水管沿轴向切半，内半径6.5 mm（内径13 mm）；
- 两个112 mm挡边距离和155 mm零点到C轴距离的测量误差；
- 水管水平时 RS00 的编码器角度、连杆位于哪一个装配分支；
- 主动杆、中间杆、水管和支架的质量、质心位置与转动惯量；
- 水管最大倾角；
- RS00/机构阶跃响应的延迟、上升时间和最大角速度；
- 相机实际帧率和从曝光到串口收到坐标的总延迟；
- 静止钢球时位置坐标的标准差；
- 滚动阻力系数当前采用用户给出的约值0.005，后续用自由滚动日志复核；动摩擦区间仍待辨识；
- 实际最大允许位置和安全边距。

不要先追求很复杂的控制器。应先让标称模型和实物的开环阶跃曲线接近，再比较 LQI、LADRC 或 MPC。

## RS00 控制模式建议

模型默认使用 RS00 的运控模式：

```text
目标位置 + 目标速度 + Kp + Kd + 前馈力矩
```

默认仿真初值：

```matlab
bp.rs00.motion_kp = 40;       % N*m/rad
bp.rs00.motion_kd = 2.0;      % N*m/(rad/s)，默认管体惯量下接近临界阻尼
bp.rs00.user_torque_limit = 5; % 先限制在额定力矩
bp.rs00.user_speed_limit = 9;  % rad/s，标称闭环仿真值
bp.actuator.max_rate = 0.35;   % rad/s，新机构水管端指令斜率
```

初次实物测试不要直接开放 14 N·m 峰值。先把水管固定可靠、钢球取下，在 1°、2°、3° 小阶跃下记录：

实机第一次测试建议暂时把 `user_speed_limit` 降到 `3 rad/s`；确认连杆无干涉、
RS00 电流和温度正常后，再根据钢球闭环需要逐步提高。新机构在3 rad/s限速下，
最不利位置的理论水管速度只有约`0.14 rad/s`，需要同时降低车辆加速度和目标轨迹速度。

- `mechPos`；
- `mechVel`；
- `iqf`；
- 电机温度；
- 指令发出时间和反馈到达时间。

然后用记录结果更新：

```matlab
bp.rs00.current_loop_time_constant
bp.rs00.command_delay
bp.rs00.feedback_delay
bp.rs00.joint_viscous_friction
bp.rs00.joint_coulomb_friction
bp.mechanism.motor_pivot
bp.mechanism.pipe_pivot
bp.mechanism.crank_length
bp.mechanism.coupler_length
bp.mechanism.pipe_attachment_radius
bp.mechanism.pipe_inertia_about_pivot
```

如果使用位置模式，优先 CSP；PP 位置模式包含内部轨迹规划，不适合高频改变水管目标角度。

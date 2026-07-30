# ADR-006：RS00两连杆滚球部署基线

Date: 2026-07-30

Status: Accepted for implementation planning; hardware verification pending

## Context

ADR-001建立了三板分工和延迟鲁棒LQG方向，但当时执行器、机构和相机参数尚未确认，
因此使用了60 Hz视觉、通用一阶执行器、`+-4 deg`摆角和可能由EdgeTalk运行FOC的假设。

用户随后确认：

- 执行器为RobStride RS00；
- EdgeTalk/PSoC E84通过CAN直接控制RS00；
- 机构为RS00主动曲柄、蓝色连接杆和水管摇杆组成的两连杆驱动；
- 水管有效长度250 mm、主动杆51.5 mm、连接杆65.5 mm；
- RS00轴高38 mm，支撑台不低于50 mm；
- 当前相机基线为100 Hz；
- IMU目标约500 Hz，但当前MSP固件尚未证明该数据率；
- MSP小车工程已接近调通，不应承担滚球外环或被无关重构。

## Decision

1. 保留三板分工：
   - MSPM0负责底盘本地控制和IMU/底盘状态采集；
   - Raspberry Pi负责100 Hz视觉；
   - EdgeTalk独占滚球估计、控制、两连杆逆解和RS00安全。
2. EdgeTalk不重复实现RS00内部FOC；首版沿用已冻结的CAN CSP位置目标，MIT阻抗模式仅作备选。
3. 滚球首版采用：
   - 200 Hz历史回溯OOSM卡尔曼；
   - 状态`[x,v,d]`；
   - LQI、IMU前馈和低带宽扰动补偿；
   - 端部预测保护和抗积分饱和；
   - 1 kHz M33安全门、200 Hz受限位置目标，以及250~500 Hz RS00反馈目标（以实测为准）。
4. 当前推荐合页轴高82 mm，正常水管角优先`+-5 deg`、恢复约`+-7 deg`、
   软件硬限位`+-8 deg`。加工变化必须重算机构。
5. 500 Hz IMU是目标而不是已验证事实。协议、物理链路和真实输出率必须先测量。
6. 旧Python LQR增益和压力结果保留作数值回归，不作为RS00两连杆部署增益。

## Consequences

- `K=[14.56338,3.31547,0.58093]`继续用于旧Python模型回归，但不得直接写入当前固件。
- EdgeTalk固件需要新增RS00协议适配、四杆正/逆解、雅可比检查、OOSM历史和安全状态机。
- MSP固件只新增与现有小车逻辑隔离的状态发布路径。
- 100 Hz视觉必须携带帧号和曝光时间或处理age；普通位置PID不属于本决策方案。
- 在完成摩擦、视觉时延、机构阶跃、RS00零位/方向和CAN延迟实测前，不提供自动使能固件。

## Supersedes

本ADR替代ADR-001中的以下假设：

- 60 Hz视觉；
- 通用“5号无刷电机”；
- `+-4 deg`固定摆角；
- 1 kHz通用摆杆角度内环；
- EdgeTalk可能自行执行20 kHz FOC；
- 旧LQR增益可直接作为部署增益。

ADR-001的三板所有权、时间戳视觉、EdgeTalk集中滚球控制和安全验证原则继续有效。

本ADR还把ADR-004、ADR-005和`VISION_MEASUREMENT_V1`中原来的120 Hz相机目标更新为
用户最终确认的100 Hz。ADR-004/005关于M33/M55所有权、USB/CAN链路、CSP、`SHADOW_ONLY`
和`ACTUATOR_TX=0`的已实现边界继续有效。

## References

- `docs/ai-handoffs/h-ball-control.md`
- `docs/architecture/system-overview.md`
- `docs/hardware/measured-parameters.md`
- `shared/protocol/README.md`
- `firmware/edgetalk/README.md`
- `experiments/h_ball_control_simulink/README.md`

# H题车载滚球控制交接

AI identity: Codex

Role: H-ball EdgeTalk USB/CAN/LQG integration

Updated: 2026-07-30

## 当前结论

正式控制链冻结为“天猛星MSPM0G3507 + 树莓派 + EdgeTalk M33/M55 + RS00”，F407不进入，NanoPi-M5只作树莓派性能或接口不达标时的单机备选。代码按协议、物理接口、数据质量、统一快照、双核IPC、算法、安全门和显示分层；`PSOC_E84_robot`仅作为CAN和工具链参考，不迁移机械臂业务或参数。

当前分支为`prep/2026`，未创建或填充`main`。所有自动测试保持`MOTOR_COMMAND_TX=0`、`ACTUATOR_TX=0`，M55只发布`SHADOW_ONLY`；本轮未烧录任何镜像、未使能电机、未启动车辆运动。

Git同步状态：已将远端3个MSPM0循迹提交作为基底无冲突rebase，本地提交和`92 passed`回归均完成；截至本记录更新时，到`github.com:443`的网络连接连续失败，普通`git push origin prep/2026`尚未完成。禁止force push；网络恢复后直接重试该命令即可。

## 分层数据路径

```text
树莓派120 Hz灰度ROI/轮廓/圆心
  -> 64 B BALL_MEASUREMENT_V1
  -> M33 emUSB流解析/CRC/序号/age

MSPM0/RS00 1 Mbps Classic CAN
  -> M33 ID/DLC/IDE/RTR/序号/健康位/freshness
  -> M33 200 Hz sensor_snapshot
  -> 共享SRAM IPC
  -> M55 FreeRTOS 200 Hz延迟观测器/LQG
  -> control_shadow(SHADOW_ONLY)
  -> M33 1 kHz安全监督
  -> ACTUATOR_TX=0
```

M33是USB、CAN、输入有效性和最终安全门的唯一所有者；M55不链接CAN驱动或执行器代码。LVGL数据钩子为10 Hz，只显示球位置/速度、IMU、偏航角速度、电机反馈、LQG目标、age和安全状态。

## 频率与电机模式

- 树莓派正式识别目标：120 Hz；64字节有效负载为7.68 kB/s。
- USB链路验收：240 Hz、15.36 kB/s；500 Hz只作合成压力档。
- MSPM0加速度/角速度：200 Hz；姿态、轮速：100 Hz；心跳：20 Hz。
- M33统一快照：200 Hz；M55 LQG：200 Hz；M33安全监督：1 kHz；LVGL：10 Hz。
- RS00反馈目标：250~500 Hz，实际周期待台架测量。
- 首版电机模式：RS00 CSP位置模式。M55输出摆杆目标角，M33负责`+-4 deg`限幅、`80 deg/s`限速、freshness和故障门，未来以200 Hz写入受限`loc_ref`并设置保守`limit_spd/limit_cur`。
- MIT位置-速度阻抗模式只作为CSP实测带宽不足时的备选；纯速度会积累角度漂移，纯力矩/电流对模型和失联保护要求更高，不作为首版。
- RS00内部闭合位置环和FOC；EdgeTalk不重复实现20 kHz三相电流环，内部频率在获得厂家资料或实测前不写死。

## 已完成代码切片

最近关键提交：

- `e8226c7 docs(edgetalk): record CM55 build evidence`
- `fb9ad82 docs(arch): freeze EdgeTalk runtime and CSP boundaries`
- `6d1cd4d fix(edgetalk): satisfy CM55 FreeRTOS build contract`
- `733feb6 test(control): trace CAN snapshots into LQG`
- `f4b01f6 perf(vision): bound USB writes and report link speed`
- `6cd3fad feat(can): gate MSPM0 samples by sequence and health`
- `ce7f097 feat(edgetalk): run M55 shadow on official FreeRTOS`
- `df47a7c docs(vision): freeze 120 Hz measurement boundary`
- `6c3feb6 feat(telemetry): measure USB and CAN stream rates`

CAN到算法的主机端到端测试会构造MSPM0/RS00帧，经CAN解码、健康/freshness门、统一快照进入LQG。偏航角速度会改变shadow目标；撤掉心跳`IMU_VALID`后，算法仍保持有限数值，但`safety_eligible=false`。

## 验证证据

- 完整纯软件回归：`python -m pytest firmware/edgetalk/tests vision/raspberrypi/tests experiments/h_ball_control_sim/tests -q` -> `92 passed`。
- M33真实ARM集成构建：`text=216884 data=14932 bss=245233`。
- 已有USB CDC实机证据：树莓派以High-Speed枚举，30秒100 Hz文本探针`2960/2960`，零超时，P95 RTT `2.14 ms`；该证据不等于240 Hz二进制端到端通过。
- 已有RS00只读证据：1 Mbps Classic CAN Get_ID收到扩展回复，错误计数为0；MSPM0五类数据仍未在真实总线上对拍。

## CM55真实构建状态

离线临时工程已完成180/180源文件编译和单核链接：

- ELF：`D:\2026_TI\tmp\CdcEchoOfficial\proj_cm55\build\APP_KIT_PSE84_EVAL_EPC2\Debug\proj_cm55.elf`
- 大小：664,588 bytes
- SHA-256：`005A5BB383F6D6C57BC4FCF34E6D56B43D82B3CE0F5992B6A651529CCF96DD73`
- 架构：ARMv8.1-M Mainline、hard-float，`-mcpu=cortex-m55+nomve`；当前MVE明确禁用。
- Device Configurator必须同时传PSE84 DSL和全局`device-db release-v4.38.0`；4.21.0不含PSE846。

该ELF不能烧录：官方示例内存图把`.hball_ipc_shared`放在`0x262FC000`，而当前M33 RT-Thread map为`0x261C0000`；两核IPC地址不一致。最终多核合并还缺匹配的`proj_cm33_ns.hex`。下一步必须复用当前M33的同一份Device Configurator内存设计，重新生成CM55链接图并同时核验两侧`start=0x261C0000/end=0x261C0100`。

## 未完成与下一步

1. 统一M33/M55 PSE84内存设计，生成匹配的CM55单核ELF和完整多核包；完成地址、架构、段大小、入口和签名检查前不烧录。
2. 在电机/底盘动力断开、轮子/执行器卸载、仅调试器/USB供电、操作员可拔线断电条件下，实测240 Hz二进制视觉；主机和M33计数必须对拍，CRC/重复/乱序/空洞为0。
3. 用户给出MSPM0和EdgeTalk具体引脚后，冻结CAN收发器使能、终端、电平、IMU UART和坐标系，再做MSPM0五类只读CAN对拍。
4. 当前树莓派脚本是合成64字节压力流；真实120 Hz灰度ROI、轮廓筛选、圆心拟合和采集时间戳还需接入同一协议。
5. 重新标定H题机构的电机零点、方向、传动比、机械限位、CSP上升时间/延迟/死区和钢球摩擦；禁止复用机械臂5号槽参数。
6. 240 Hz USB、MSPM0多流CAN、M55共享地址和CSP台架逐项通过后，才能设计人工使能的执行器适配层；自动测试永久禁止运动发送。

## 工作树保护

以下未跟踪内容属于用户既有工作，本轮没有暂存或提交：

- `experiments/h_ball_control_sim/hballsim/full_vehicle_simulation.py`
- `experiments/h_ball_control_sim/hballsim/vehicle_campaign.py`
- `experiments/h_ball_control_sim/hballsim/vehicle_dynamics.py`
- `experiments/h_ball_control_sim/hballsim/vehicle_parameters.py`
- `experiments/h_ball_control_sim/tests/test_full_vehicle_*.py`
- `experiments/h_ball_control_sim/tests/test_vehicle_*.py`
- `firmware/edgetalk/config/`
- `tmp/`

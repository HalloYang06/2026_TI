# Q3与任务状态机实机调试复盘（2026-07-31）

## 结论

Q3实物闭环已用同一套部署PID完成两条路径验证：

- FinSH单独启动：`0 → +50 mm → -50 mm`，4.024 s，最终约-45 mm，PASS。
- 模拟TI SW1后的真实CAN状态机链路：进入RUNNING，抵达+50 mm后折返，最终
  -51 mm，最终误差1 mm，`passed=1`。

部署参数固定为`Kp=0.70, Ki=0.15, Kd=0.35`，静摩擦补偿0。Q3禁止使用车体
IMU前馈；IMU补偿只用于Q4～Q6车载保持。

## 系统边界

- TI MSPM0G3507：SW3选题、SW1启动、IMU/底盘数据、任务CAN客户端。
- EdgeTalk M33：唯一任务仲裁器、RS00准备/使能、安全门和Q3 PID。
- 树莓派：约100 Hz视觉位置，经USB CDC发送；同时接收只读控制日志。
- RS00 5号电机：CSP位置模式，标定水平约1.721 rad。

## 发现并修复的问题

### 1. TI显示MISS M33

TI的CAN发送正常，但接收中断没有进入，RAM中`rx_total=0`。在1 ms任务加入FIFO0
轮询兜底后，TI收到`0x082/0x083`。该兜底不改变CAN协议。

### 2. RESET导致永久STABILIZE

任务终止后TI持续发送RESET。旧M33每帧都清除500 ms READY稳定计时，且M33重启时
没有上下文会拒绝RESET。修复为：

- 无上下文RESET建立PREPARING上下文；
- 同epoch/任务下，PREPARING和READY中的重复RESET幂等；
- 终态RESET仍能进入新的PREPARING。

### 3. RUNNING假卡

Q3在5秒内失败后会回平并停止，但旧任务仲裁器没有把失败转换成ABORTED，屏幕一直
RUNNING。现在`phase=4 && active=0 && !passed`会以DEADLINE原因终止。

### 4. 状态机启动而PID不动作

单独命令会在ARM后等待约120 ms再启动Q3；状态机则在ARMED第一帧立即检查
`mechPos/mechVel`读回。该瞬间参数尚未刷新，旧逻辑直接ABORT。修复为在
START_PENDING/ARMED期间等待并重试，直到读回新鲜；2秒运动看门狗仍保留。

### 5. Q3误混入IMU补偿

车体IMU补偿曾无条件叠加到Q3，改变了已调好的静态PID。现在`mode=1`的Q3明确禁用
IMU项，每次Q3启动强制恢复0.70/0.15/0.35和零摩擦boost。

### 6. LCD显示Q3但M33残留Q4

协议中的`mission_id`从题目2开始编号，因此日志`q=3`代表第四问，不代表第三问：

| `mission_id` | 题目 |
|---:|---|
| 2 | Q2 |
| 3 | Q3 |
| 4 | Q4 |
| 5 | Q5 |
| 6 | Q6 |

故障发生时TI LCD显示的是TI本地`selected_mission=Q3`，M33权威状态也可能仍是
旧`mission_id=3/Q3/RUNNING`，但两者epoch不同。TI单独复位会把本地上下文恢复为
`epoch=1/Q2`并发送
PREPARE；旧M33处于RUNNING时会拒绝跨任务PREPARE，双方无法重新同步。LCD仍能显示
本地选择，造成“界面是Q3、实际残留Q4”的误导。

修复后的恢复握手为：

1. TI启动时先发送RESET，不直接发送PREPARE。
2. M33收到RESET时，如果旧任务处于START_PENDING、RUNNING或FINISHING，先进入
   CONTROLLED_ABORT并保持旧上下文一个状态周期。
3. TI周期发送的下一帧RESET建立新的`epoch + mission_id`上下文。
4. M33通过`0x082`状态帧回报完全相同的epoch和任务，并稳定满足READY 500 ms。
5. TI只有在状态帧新鲜度不超过150 ms、epoch匹配、任务匹配且required ready mask
   全部满足时，才显示ALL READY并接受SW1。
6. 上电RESET尚未收到匹配`0x082`前，SW3和SW1都锁定，防止本地选择把恢复RESET
   提前覆盖成PREPARE或START。
7. `0x081`任务意图每50 ms获得两个相邻发送机会，并优先于普通IMU遥测，避免共享
   TX buffer忙时丢掉唯一发送槽而导致RESET/PREPARE长期饥饿。
   在尚未收到任何匹配状态时，所有非遥测发送槽都用于重复RESET，直到首次同步完成。

RESET被定义为跨MCU恢复命令，因此允许其epoch小于M33旧epoch；普通PREPARE和START
仍执行严格的回绕安全epoch检查。这样只复位TI或只复位M33都能重新收敛，同时不会把
旧RUNNING任务直接伪装成新任务READY。

如果只复位M33，而TI仍在重发已经确认过的START，新M33会先用该START携带的
`epoch + mission_id`重建PREPARING上下文，但不会立即启动；READY条件稳定500 ms后，
后续重复START才进入START_PENDING。这修复了M33重启后无上下文而永久拒绝START，
同时保留完整安全门。

CAN同步帧职责：

- `0x081` TI→M33：`epoch、mission_id、command、event_time`任务意图；
- `0x082` M33→TI：`epoch、mission_id、global_state、ready_mask、reason、sequence`
  权威任务状态；
- `0x083` M33→TI：界面和诊断信息，不作为SW1启动许可的唯一依据。

实机回归结果：分别重新烧录M33和TI后，TI从无有效状态启动，重复发送RESET；M33
最终显示`valid=1 epoch=1 q=2 state=PREPARING`，任务意图接收计数从0增长到56，
状态发送失败为0。调试器读取TI客户端RAM得到相同的`epoch=1、mission_id=2`和有效
状态，证明双方已从旧上下文自动收敛。此时尚缺少的READY位仍按真实视觉、RS00和链路
状态显示，不会被同步逻辑伪造为ALL READY。

后续Q3复现确认`reason=9`并不是PID停在错误位置。诊断输出为
`mask=0x06 vision_age=251 motor_pos_age=8 motor_vel_age=3`：M33 USB重连时树莓派视觉
连续中断约251 ms，而RS00位置和速度读回正常。旧代码遇到单次50 ms视觉间隙即停，
第一次修正的200 ms连续失效门限仍不足。当前门限为连续500 ms；短暂间隙保持上一条
已经限幅的管角命令，超过500 ms才停机，钢球物理越界仍立即停机。日志会保留具体
失效mask和各数据年龄，后续不得把此类安全停机误判为PID稳态误差。

继续实机追踪发现树莓派当时运行的是旧版单向桥接，只写视觉帧、不读取M33在Q3期间
上报的50 Hz控制日志。USB CDC IN端点积压后，树莓派出现`Write timeout`并等待重连，
M33侧表现为视觉在223帧后完全停止、`vision_age`增长到数十秒。已部署新版
`edgetalk_camera_bridge.py`和`edgetalk_log_protocol.py`到
`/home/halloyang/hball-edgetalk-camera/`：每次写视觉后排空控制日志，并取消可能在USB
重枚举时无限等待的逐帧`flush()`。systemd用户服务重启后，实跑Q3期间视觉持续约
115 Hz，M33成功发送265条控制日志且`telemetry_drop=0`，5秒全过程未再触发reason=9。

调试器模拟按键必须在RAM写入后执行pyOCD `go`。`pyocd cmd`连接目标时会halt内核；
若只写`command/start_requested`而不恢复运行，TI CAN上报会停止，产生人为的状态残留。
最后一轮完整模拟使用“Q2同步→SW3等价写入Q3→go→Q3 READY→SW1等价START→go”。
该轮因钢球起点已滚到约+38 mm而不满足题目从O点开始，5秒时仍在+39 mm、Q3判FAIL；
它只用于验证状态机和数据链路，不能作为PID性能结论。正式Q3复测前必须手动将球放回
O点。

## “模拟按键”的方法

没有拉低PA08，也没有改变GPIO。调试器直接把TI任务客户端设置为按键处理完成后的
同一状态：

```text
command = START
start_requested = true
```

之后TI仍通过真实1 Mbps CAN发送`0x081 START`；M33任务仲裁、RS00准备/使能、
视觉PID、状态回传全部走真实硬件链路。因此该方法只跳过GPIO消抖，适合定位按键之后
的分布式状态机，不等价于验证按键电气和消抖本身。

## 日志—Simulink闭环

Q3运行时M33以最高50 Hz提交`HBLG`实控记录，并置`status_flags.Q3_ACTUAL`。记录包含：

- 实测/估计球位、估计速度、Q3目标位置；
- 实际管角命令；
- RS00机械角和速度；
- 车体纵向加速度、俯仰；
- Q3 phase、active、passed。

树莓派用`edgetalk_camera_bridge.py --telemetry-log`落盘原始CRC帧，再用
`control_log_to_csv.py`转换。MATLAB用`import_edgetalk_control_log.m`生成
timeseries，`analyze_edgetalk_q3_log.m`复算题目指标。

## 下一步调参顺序

1. 每次Q3/Q4实跑保存独立`.hblg`，不要把多轮追加到同一文件后直接当一轮分析。
2. 先在Simulink用实测管角驱动plant，对齐球位响应，辨识延迟、滚阻和摩擦。
3. Q3保持当前已通过PID作为回退基线。
4. Q4～Q6再单独加入IMU前馈，先A到B，后整圈；不得把未验证参数回灌Q3。
5. 参数修改必须记录固件commit、摄像头零点、球起点、最终误差和完成时间。

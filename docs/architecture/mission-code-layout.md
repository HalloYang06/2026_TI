# H题任务代码分层与函数调用关系

Date: 2026-07-31

Status: accepted code-layer design

## 1. 唯一状态所有权

| 状态/能力 | 唯一所有者 | 其他节点只做什么 |
|---|---|---|
| Q2～Q6全局阶段、deadline、完成/失败 | EdgeTalk M33 | MSP/M55/Pi上报事实并执行命令 |
| 球状态估计与滚球控制 | EdgeTalk M55通用算法 | M33给目标和授权，M55回报结果 |
| READY、epoch、START幂等、安全授权 | EdgeTalk M33 | MSP产生操作员意图 |
| 底盘速度环、循迹、A/B/停稳检测 | MSP通用底盘服务 | M33发送动作，MSP上报事件 |
| 圆心、置信度、录像、回放 | Pi通用服务 | M33发送marker，Pi返回ACK |
| RS00最终命令发送 | M33安全门后的适配器 | M55只发布候选控制结果 |

因此“大部分任务逻辑在EdgeTalk”具体指：M33拥有全部评分任务阶段机，M55拥有全部
滚球实时算法；MSP和Pi没有五套完整任务逻辑。

## 2. 代码层，不是简单目录层

```text
L5  M33 Q2～Q6专用任务状态机
    只做 phase / target / deadline / complete / abort 决策

L4  节点公共运行时
    M33 mission runtime、M55 realtime runtime、MSP chassis executor、Pi service runtime

L3  公共业务能力
    READY计算、事件锁存、任务IPC、录像marker、底盘动作、控制安全资格

L2  通用纯算法
    视觉速度估计、OOSM Kalman、LQR/LQI、前馈、稳定判据、四连杆、限幅

L1  协议与平台适配
    CAN/USB/IPC编解码、RTOS同步、时间戳、缓存维护

L0  驱动
    MCAN、UART、USB CDC、GPIO、PWM、RS00帧发送
```

允许调用方向是`L5 → L4 → L3 → L2`以及运行时对`L1/L0`的适配调用。L2不得知道Q号；
L0/L1不得调用L4/L5做业务决策。接收回调只把数据写入队列/快照，不能在中断或CAN解析
函数中推进任务阶段。

## 3. M33主调用链：任务大脑

现有`hball_worker_entry()`保留为RT-Thread周期入口，但逐步收敛为以下顺序：

```c
static void hball_m33_worker_tick(uint32_t now_ms)
{
    hball_transport_poll();                 /* L1: 只收发/解码 */
    hball_m33_facts_snapshot(&facts);       /* L3: 原子快照 */
    hball_ready_evaluator_step(&facts, &ready);

    hball_mission_runtime_step(             /* L4: 唯一全局编排入口 */
        &runtime, &facts, &ready, now_ms, &actions);

    hball_mission_context_publish(&actions.m55_context); /* IPC */
    hball_chassis_request_publish(&actions.chassis);     /* CAN */
    hball_marker_publish(&actions.marker);               /* USB */
    hball_mission_status_publish(&actions.status);       /* CAN */
}
```

`hball_mission_runtime_step()`内部只允许一个题号分发点：

```c
const hball_m33_task_ops_t *ops = hball_m33_task_lookup(runtime->mission_id);
ops->step(&runtime->task_state, facts, now_ms, actions);
```

每道题实现同一接口，专用文件只存在于M33：

| 题目 | M33专用函数 | 主要决策 |
|---|---|---|
| Q2 | `hball_m33_q2_step()` | 离A、完整圈、重捕获A、刹车、20 s deadline |
| Q3 | `hball_m33_q3_step()` | HOLD O→目标+5→稳定→目标-5→稳定、5 s deadline |
| Q4 | `hball_m33_q4_step()` | A→B、B通过/停车口径、中心目标、8 s deadline |
| Q5 | `hball_m33_q5_step()` | 中心保持、完整圈、A重捕获、30 s deadline |
| Q6 | `hball_m33_q6_step()` | START时一次锁存目标、整圈保持、禁止重锁存 |

这些函数只填充`hball_mission_actions_t`，不能直接调用CAN、USB、IPC或RS00发送函数。
公共`hball_mission_runtime_step()`在函数返回后统一发布，便于幂等、限频和测试。

### M33任务函数看到的输入

`hball_mission_facts_t`只包含已经验证和带age的事实：

- MSP：A/B事件、线丢失、轮速、底盘停稳、失联/故障；
- M55：估计位置/速度、settled、controller ready、fault、输出age；
- Pi：球位置/置信度、录像READY/ACK、USB age；
- RS00：模式、位置/速度、故障、反馈age；
- 安全：急停、配置hash、各链路freshness。

Q函数不能读取各驱动的全局变量，避免同一次step看到撕裂数据。

## 4. M55调用链：无题号的200 Hz通用控制

M33和M55之间是IPC，不是函数调用。M55每5 ms执行：

```c
static void hball_m55_control_tick(float dt_s)
{
    hball_m55_read_sensor_snapshot(&sensors);
    hball_m55_read_mission_context(&mission);  /* 含epoch/phase/target/limits */

    hball_control_pipeline_step(
        &pipeline,
        &sensors,
        dt_s,
        mission.target_position_m,
        &control);

    hball_settle_detector_step(
        &settle,
        control.estimated_position_m - mission.target_position_m,
        control.estimated_velocity_mps,
        dt_s,
        &settled);

    hball_m55_result_publish(&mission, &control, settled); /* SHADOW_ONLY */
}
```

现有`hball_control_pipeline_step()`已经显式接收`target_position_m`，因此接入任务时只把
当前硬编码`0.0F`替换成经过epoch/freshness检查的`mission.target_position_m`。函数内部
继续调用通用估计、LQI和四连杆，不增加`switch (mission_id)`。

Q3阶段顺序和Q6目标锁存都在M33；M55只报告“当前目标是否稳定”。这样算法函数对五道题
完全相同，便于用同一仿真和golden trace验证。

## 5. M33安全门调用链：任务函数不能直接驱动电机

安全任务独立于M33任务状态机，建议1 kHz运行：

```c
hball_control_shadow_read(&shadow);
hball_mission_authority_snapshot(&authority);
hball_m33_facts_snapshot(&facts);

eligible = hball_control_guard_observe(
    &guard, &shadow, &facts.sensors, now_ms, &reason);

if (eligible && authority.actuator_allowed)
    hball_rs00_control_apply_guarded(&shadow, &authority);
else
    hball_rs00_control_safe_stop(reason);
```

当前仍固定`authority.actuator_allowed=false`和`ACTUATOR_TX=0`。以后开放时也只能修改安全
授权层，Q2～Q6函数和M55控制器不获得RS00发送API。

## 6. MSP调用链：操作入口和通用底盘执行器

MSP不实现Q2～Q6全局状态机。按键和底盘分开：

```text
get_task_key_event()
  -> hball_msp_mission_ui_handle_key()
  -> hball_mission_client_select/request_start()
  -> CAN intent 0x081

CAN chassis request
  -> hball_chassis_executor_accept(action, epoch, profile)
  -> hball_chassis_executor_step(track, wheel, now)
  -> set left/right target speed
  -> CAN chassis events 0x084
```

M33下发的是通用动作，例如`INHIBIT`、`FOLLOW_ROUTE`、`BRAKE`、`HOLD_STOP`，不是让MSP
运行一个完整Q5状态机。MSP必须保留本地失联停车、PWM/速度环和传感器事件锁存；这些是
实时安全能力，不是全局任务决策。

## 7. Pi调用链：一套服务处理所有题

```text
camera frame
  -> contour/ROI center detector
  -> vision measurement USB

mission marker USB
  -> generic recorder handle_marker(epoch, q, START/END/ABORT)
  -> prebuffer/finalize/playback
  -> marker ACK USB
```

Pi不需要`q2.py`到`q6.py`五个状态机。题号只决定文件名、显示标签和profile字段；录像和
视觉算法保持一套。Q1是这套服务的常驻展示/回放能力。

## 8. 通用与专用的判定

| 代码 | 类型 | 原因 |
|---|---|---|
| Kalman/OOSM、视觉多帧速度、LQR/LQI、前馈、四连杆 | 通用算法 | 输入目标即可服务Q2～Q6，不知道题号 |
| settled判据、斜率/角度/边缘限幅 | 通用算法 | 由config控制，不包含阶段顺序 |
| READY聚合、epoch幂等、marker重发 | 公共运行时 | 多题复用，但属于M33系统服务 |
| `+5 cm后切-5 cm` | Q3专用M33逻辑 | 是评分阶段顺序，不是控制算法 |
| START时锁存初始球位置一次 | Q6专用M33逻辑 | 是任务语义，不能放估计器里 |
| A重捕获后刹车 | Q2/Q5/Q6各自M33逻辑 | deadline/profile不同，不隐式共享状态 |
| PWM、CAN、USB、RS00帧 | 平台/驱动 | 只执行，不做任务决策 |

纯算法API必须显式传入配置，禁止用Q号选择参数：

```c
hball_algorithm_status_t hball_ball_controller_step(
    hball_ball_controller_t *controller,
    const hball_ball_controller_config_t *config,
    const hball_ball_controller_input_t *input,
    hball_ball_controller_output_t *output);
```

调用者先由M33 profile和context确定配置/目标，再调用算法。算法返回结果，不能回调任务
状态机。

## 9. 代码承载位置

目录服务于上述调用层，不代表四端各写一份任务：

| 代码层 | 位置 |
|---|---|
| M33通用mission runtime/arbiter | `firmware/edgetalk/src/hball_mission_*.c` |
| M33 Q2～Q6专用step | `firmware/edgetalk/rtthread/missions/` |
| M55通用200 Hz runtime | `firmware/edgetalk/freertos/hball_m55_shadow_task.c`及后续runtime |
| EdgeTalk通用估计/控制 | 当前`firmware/edgetalk/src/hball_*control*.c`；跨平台稳定后提升到`shared/algorithms/` |
| MSP按键/UI/mission client | `firmware/mspm0/wit-oled-hardware-spi/App/Mission/` |
| MSP通用底盘执行器 | `firmware/mspm0/wit-oled-hardware-spi/App/Chassis/` |
| Pi通用mission/录像服务 | `vision/raspberrypi/mission/` |
| 协议 | `shared/protocol/` |
| 未验证算法/压力测试 | `experiments/` |

## 10. 评审时必须检查

- 只有M33 Q文件出现Q2～Q6专用阶段枚举和转移。
- `hball_control_pipeline_step()`及其下层搜索不到Q号和传输API。
- CAN/USB接收函数只解码并发布事实，不直接推进任务。
- MSP/Pi没有复制全局任务完成条件。
- 跨核/跨板调用全部经过带epoch、sequence和freshness的协议。
- 执行器发送只有M33 safety guard后的一个出口；shadow构建保持关闭。

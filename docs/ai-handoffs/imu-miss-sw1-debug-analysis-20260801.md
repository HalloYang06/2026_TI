# MSPM0 IMU周期MISS与SW1偶发启动失败调试分析（2026-08-01）

## 结论

IMU并非随机断线。实机控制日志显示，`IMU valid`以约1.42 s为周期失效，每次主要持续
0.39～0.46 s。Q4～Q6的SW1启动必须使用M33最新的`READY`状态，而`READY`又要求IMU有效；
因此在失效窗口内按SW1会被TI本地启动门拒绝，表现为“多按几次才能启动”。

根因位于MSPM0协作式运行时的积压处理：调度器能够累计最多3个IMU release，但前台一次
`poll`会先取走全部release，随后只调用一次`WIT_Service(32)`。LCD刷新或其他前台工作造成
短时延迟时，已累计的多个解析机会被错误合并成一次，WIT字节队列不能按积压量排空，最终
引起周期性数据陈旧和`IMU MISS`。

提交`a5f6fd3`改为对每个已取出的IMU release各执行一次有界解析。单次`poll`最多解析
`3 × 32 = 96 B`，不会无界补跑。CAN任务仍保持一次合并执行，避免用同一个`now_ms`
重复运行时隙调度。

## 现象与实机证据

采集运行ID：

```text
.codex_tmp/q6_runs/run_20260801_145552_live_q6_auto_target/
```

目录名沿用了采集器的Q6命名，但帧内`control_mode=0x0101`、目标为-50 mm，实际是正在运行
的Q3控制记录。因此该轮只用于分析共享的MSP/IMU数据链路，不作为Q6控制效果证据；原始
运行目录也未提交到Git。

该轮HBLG日志共4313条有效控制记录，持续93.234 s，控制日志平均46.25 Hz。日志中的
`sensor_valid_flags`按位定义为：

| 位 | 含义 |
|---:|---|
| 0 | 视觉有效 |
| 1 | IMU有效 |
| 2 | 轮速有效 |
| 3 | RS00电机有效 |
| 4 | MSP心跳有效 |

统计结果：

| 指标 | 实测值 |
|---|---:|
| IMU有效比例 | 69.19% |
| IMU失效事件 | 65次 |
| 主要失效持续时间 | 0.39～0.46 s |
| 主要失效起点间隔 | 约1.42 s |
| 最长观测失效 | 约0.46 s |

`sensor_valid_flags`分布如下：

| flags | 条数 | 状态解释 |
|---:|---:|---|
| `0x1F` | 2948 | 视觉、IMU、轮速、电机、心跳全部有效 |
| `0x09` | 848 | 仅视觉和电机有效 |
| `0x19` | 226 | 视觉、电机、心跳有效 |
| `0x0D` | 164 | 视觉、轮速、电机有效 |
| `0x1D` | 91 | 仅IMU无效 |
| `0x0F` | 36 | 仅MSP心跳无效 |

这说明故障不只是LCD文案抖动：M33传感器快照中的有效位确实周期下降，而且部分窗口同时
影响MSP侧的IMU、轮速和心跳新鲜度。

典型IMU失效区间：

```text
1.099～1.513 s
2.515～2.951 s
3.930～4.327 s
5.350～5.743 s
6.770～7.206 s
8.189～8.587 s
```

相邻起点约为1.42 s，排除了“偶尔接触不良”这一类纯随机解释，优先指向固定容量、固定
预算或计数回绕相关的软件路径。

本轮HBLG没有同步记录TI端的`wit_queue_depth/dropped`，所以日志本身只能证明周期失效；
“积压release被错误合并”由代码检查直接确认，二者的最终因果闭环仍需重新供电后的队列
计数和修复后HBLG对比完成。

## 数据率和预算分析

JY901S配置为加速度、角速度和姿态角三类报文，每类约200 Hz。每帧11 B，因此实际业务
数据率约为：

```text
3 × 11 B × 200 Hz = 6600 B/s
```

等价于每5 ms约进入33 B。UART为115200 bit/s，按8N1计算，链路理论上限为11520 B/s，
即每5 ms最多57.6 B。这里需要区分：57.6 B是线路容量上限，33 B才是当前三类报文的
典型有效负载。

WIT前台解析预算为每次32 B：

```c
#define WIT_FOREGROUND_BUDGET_PER_SERVICE 32U
```

正常每1 ms轮询时该预算足够；问题出现在前台被LCD绘制等工作短时占用后。1 ms调度器会把
未处理release累计到`HBALL_COOP_MAX_PENDING == 3`，但旧代码先把三个release全部取走，
最后只调用一次32 B解析：

```c
while (imu_consumed < HBALL_COOP_MAX_PENDING && take(IMU)) {
    imu_consumed++;
}
if (imu_consumed != 0U) {
    imu_service(...);  /* 旧代码无论积压多少都只调用一次 */
}
```

因此调度器记录了积压，却没有把积压转换为相应的解析预算。ISR仍持续向256 B单生产者/
单消费者队列写入，长期重复后产生队列高水位、丢字节、解析帧停止更新，随后CAN上的IMU
源序号和M33新鲜度超时。

## SW1为什么同时受影响

SW1不是直接使能电机。TI的任务客户端仅在以下条件成立时接受START：

```c
client->status_valid
&& !client->start_requested
&& client->latest_status.global_state == HBALL_MISSION_STATE_READY
&& status_age <= 500 ms
```

Q4～Q6的任务`required_mask`包含`HBALL_MISSION_READY_IMU`。M33发现IMU不满足新鲜度后会
把`READY`退回`PREPARING`并通过CAN状态帧通知TI。此时按下SW1，
`hball_mission_client_request_start()`返回`false`，按键事件已经消费但START没有锁存。
等下一段IMU有效窗口到来后再次按键才会成功，所以操作者感觉必须多按几次。

Q3的任务策略不要求IMU READY，因为Q3静态滚球明确禁用车体IMU前馈；因此本问题主要影响
Q4～Q6。不能通过删除Q4～Q6的IMU安全门来掩盖数据链路故障。

## 修复

修改文件：

- `firmware/mspm0/wit-oled-hardware-spi/App/Runtime/hball_runtime_dispatcher.c`
- `firmware/mspm0/wit-oled-hardware-spi/tests/hball_runtime_dispatcher_host_tests.c`

修复后，每个已经从调度器取出的IMU release都兑现一次32 B解析预算：

```c
for (release = 0U; release < imu_consumed; ++release) {
    dispatcher->hooks.imu_service(dispatcher->hooks.context, now_ms);
}
```

该设计有三个边界：

1. `imu_consumed`最多为3，因此一次`poll`最多处理96 B，不会形成无界追赶循环。
2. IMU只解析已经由ISR放入队列的字节，不回放传感器时间戳，不会生成重复测量。
3. CAN仍只服务一次。CAN发送槽由`now_ms % period`决定，用同一个时间重复调用不能恢复过去
   的发送槽，反而可能重复发送或增加TX busy。

对应主机测试从“积压合并为一次”改为验证“CAN合并一次、IMU按三个有界release执行三次”。

## 已完成验证

- Keil构建成功：`0 Error(s)`。
- 新HEX已通过Horco CMSIS-DAP烧入MSPM0G3507并复位。
- 修复提交：`a5f6fd3 fix(mspm0): drain pending IMU work`。
- 分析工具提交：`3986928 feat(simulink): analyze live Q6 telemetry`，目标位置从每条M33
  HBLG记录自动读取，不使用人工填写的目标值。

烧录后现场随即断电，因此尚未完成修复后的长时间实机日志回归。当前结论是“代码根因已修、
构建和烧录完成”，不能把它写成“实机长跑已经通过”。

## 重新供电后的回归步骤

1. 进入任务菜单，静置60 s，不按SW1，LCD不得周期显示`MISS: IMU`。
2. 读取WIT统计量，稳定窗口内要求：
   - `wit_queue_dropped_byte_count`增量为0；
   - `wit_queue_depth`能够回到0；
   - 三类帧计数连续增长，速率接近各200 Hz。
3. 选择Q4或Q6，完成PB21调平/目标设置；LCD稳定显示READY后只按一次SW1，必须进入
   START_PENDING/RUNNING，不允许靠重复按键碰有效窗口。
4. 树莓派连续采集至少60 s HBLG，使用：

   ```text
   experiments/h_ball_control_simulink/prepare_edgetalk_q6_run.py
   experiments/h_ball_control_simulink/analyze_edgetalk_q6_log.m
   ```

   自动生成`q6_simulink.csv`和`q6_metrics.json`。
5. 验收目标：`imu_valid_fraction >= 0.99`、无持续超过20 ms的周期性IMU失效、SW1单次启动
   成功。若仍有MISS，再同时读取`wit_queue_depth/high_water/dropped`和运行时
   `imu_deadline_miss_total`，区分UART队列问题与前台长阻塞。

## 禁止的规避方式

- 不删除Q4～Q6的IMU READY位。
- 不把20 ms控制新鲜度直接放宽到数百毫秒来隐藏陈旧数据。
- 不用无限循环一次性排空整个队列，避免IMU解析反过来饿死CAN和底盘任务。
- 不把SW1改成无条件直接使能电机；START仍必须经过TI/M33 epoch状态机和安全门。

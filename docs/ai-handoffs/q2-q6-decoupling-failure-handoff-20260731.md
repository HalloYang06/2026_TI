# Q2-Q6 解耦与当前失败状态交接（2026-07-31 18:00）

## 1. 接手后先看这一节

仓库与分支：

```text
GitHub: HalloYang06/2026_TI
本机:   C:\Users\ASUS\Desktop\diansai\2026_TI
分支:   prep/2026
HEAD:   7d1ee35
```

当前 TI 板已经烧入 `7d1ee35` 对应的累计镜像。烧录、reset、go 均返回成功，
但用户实机观察到 **LCD 完全不亮**。本线程按用户要求已经停止继续修改和烧录。
因此：

1. 不要把当前 HEAD 当作可用固件。
2. 不要继续在当前故障镜像上叠加 PID、灰度极性或电机方向补丁。
3. 先恢复已知可用原始镜像，再用独立 worktree 做逐提交硬件二分。

当前工作区还有不属于本交接文档的修改，禁止 reset/checkout 丢弃：

```text
M firmware/edgetalk/rtthread/hball_bench_app.c
M firmware/mspm0/wit-oled-hardware-spi/Keil/wit-oled-hardware-spi.uvprojx
```

其中 `uvprojx` 是构建生成变化；`hball_bench_app.c` 属于此前 Q3/Q4 工作。

## 2. 用户最终确认的系统边界

这是后续实现的最高优先级约束，旧架构文档中“M33拥有Q2”的说法已经过时。

| 题目 | MSPM0 | CAN/英飞凌 | 底盘 | 钢球 |
|---|---|---|---|---|
| Q2 | 独立完成选题、启动、循迹、停车和计时 | 完全不参与 | 循迹一圈并停A | 不控制 |
| Q3 | SW3选题、SW1启动，发送CAN任务意图 | 英飞凌完成静止车上的 `0 → +5 cm → -5 cm` | 必须保持关闭 | 英飞凌闭环 |
| Q4 | 循迹A到B并上报底盘事件 | 从Q3开始保持通信 | MSPM0循迹 | 英飞凌稳定在O点 |
| Q5 | 循迹一圈并上报底盘事件 | 保持通信 | MSPM0循迹 | 英飞凌稳定在O点 |
| Q6 | 循迹一圈并上报底盘事件 | 保持通信 | MSPM0循迹 | 英飞凌保持指定位置 |

通信生命周期必须是：

```text
上电默认Q2 -> CAN业务静默
SW3切到Q3/Q4/Q5/Q6 -> 启用任务CAN
切回Q2 -> 再次关闭任务CAN
```

Q2 的循迹函数必须作为冻结基线，后续 Q3-Q6 调试不得改变它。

## 3. 唯一确认能正常循迹的原始固件

用户从微信文件中提供的完整原始工程：

```text
C:\Users\ASUS\Documents\xwechat_files\
wxid_z2r3xltrnsg212_a107\msg\file\2026-07\
wit-oled-hardware-spi(1)
```

关键事实：

- 其中 `main.c` 与仓库 `db1bdee`（2026-07-30 02:03）内容完全一致。
- 原始 HEX 时间为 2026-07-30 01:56。
- 原始 HEX SHA-256：

```text
81829a52252953e5f97882bd30f43fc169861e76e6e2a80b72dcf43281fbd7f7
```

- 将该 HEX 原封不动烧入 TI 后，用户明确确认：**能够正常循迹**。
- 原始固件没有新状态机。屏幕保持 `TASK1 FAST LAP`，双击旧启动键运行。

为避开 Windows 对带括号路径的解析问题，本线程曾复制同一 HEX 到：

```text
C:\Users\ASUS\Desktop\diansai\2026_TI\
firmware\mspm0\wit-oled-hardware-spi\
Keil\Objects\known-good-q2.hex
```

烧录前必须重新核对 SHA-256，不能假定该构建目录中的文件永远未被替换。

已验证可用的“复位下连接”命令：

```bat
py -m pyocd load -u 2d2670f3 -t mspm0g3507 ^
  -O connect_mode=under-reset -O reset_type=hw ^
  Keil\Objects\known-good-q2.hex

py -m pyocd cmd -u 2d2670f3 -t mspm0g3507 ^
  -O connect_mode=under-reset -O reset_type=hw ^
  -c reset -c go
```

Horco CMSIS-DAP UID：

```text
2d2670f3
```

## 4. 本线程的实机现象时间线

### 4.1 原始镜像

烧入上述 `known-good-q2.hex` 后：

```text
LCD正常
小车方向正常
循迹正常
```

这是目前唯一完整通过的 Q2 证据。

### 4.2 状态机源码重新构建后的现象

先后出现过：

- 能向前启动，但立即拐出线路；
- 原地旋转；
- 用户观察到左轮很快、右轮很慢；
- SW3 曾因内部 `status_valid` 门槛无法切题；
- 修复 SW3 后能够切题，但 Q2 仍未完成正常循迹验收。

这些现象不能简单归因于电机极性。原始 HEX 与当前源码中的
`motor_pwm_set()` 汇编语义一致，左右方向函数没有发现确定的符号翻转证据。

### 4.3 当前最终镜像

烧入 `7d1ee35` 累计镜像后：

```text
pyOCD: erase/program/reset/go 全部成功
用户观察: LCD完全不亮
```

本线程没有继续检查 PC、HardFault、初始化阻塞位置或供电，避免继续扩大改动。

## 5. 错误代码review的澄清

曾收到一份review，声称：

- 灰度极性被改成 `!= 0`；
- PIN0/PIN1 从 PB17/PB18 变成 PA26/PA27；
- TIMER0 无条件覆盖电机；
- `track.c` 中的原地旋转命令导致当前Q2旋转。

该review检查的是另一个目录：

```text
C:\Users\ASUS\Desktop\diansai\wit-oled-hardware-spi\...
```

不是当前实际构建目录：

```text
C:\Users\ASUS\Desktop\diansai\2026_TI\
firmware\mspm0\wit-oled-hardware-spi
```

对实际工程逐条核对后的结果：

- 8路灰度均使用 `DL_GPIO_readPins(...) == 0U` 判定黑线；
- PIN0/PIN1 仍为 PB17/PB18；
- PA26/PA27 是 MCAN TX/RX；
- 实际工程的 `track.c` 与原始可用工程无有效差异；
- TIMER0 电机输出受 `speed_pid_enabled == SPEED_PID_ENABLED` 门控。

因此不要按这份错误review反转灰度极性或修改灰度引脚。

## 6. 关键提交与可信度

以下提交均只存在本地分支；GitHub HTTPS 推送因本机没有凭据而失败。

| 提交 | 内容 | 实机结论 |
|---|---|---|
| `db1bdee` | 原始 Q2 `main.c` 基线 | 原始完整HEX实机正常 |
| `a4a29e1` | 撤销未验证的双轮同步启动 | 编译/烧录成功 |
| `0306eb3` | Q2不发送任务start/finish | Q2仍拐出 |
| `7cb0f98` | Q2运行时暂停CAN | 未解决，出现原地旋转 |
| `6aceaa5` | Q2运行时暂停WIT DMA/UART | 构建通过，Q2未验收 |
| `babfb9e` | 无M33状态也允许SW3选题 | 用户确认SW3可以切题 |
| `a601b70` | 新增Q2/Q3/Q4-Q6职责策略 | 仅编译验证 |
| `2735a09` | Q2禁用任务CAN，Q3-Q6启用 | 仅编译验证 |
| `7d1ee35` | Q3不进入循迹，Q4-Q6才循迹 | 烧录后LCD不亮 |

不要因为这些提交“编译通过”就把它们视为实机通过。

## 7. 当前解耦代码位置

任务职责策略：

```text
firmware/mspm0/wit-oled-hardware-spi/
App/Mission/hball_mission_policy.[ch]
```

当前定义：

```text
Q2: chassis=true,  ball=false, CAN=false
Q3: chassis=false, ball=true,  CAN=true
Q4: chassis=true,  ball=true,  CAN=true
Q5: chassis=true,  ball=true,  CAN=true
Q6: chassis=true,  ball=true,  CAN=true
```

MSPM0任务入口：

```text
firmware/mspm0/wit-oled-hardware-spi/main.c
  select_car_task()
  lap_test()
  wait_ball_only_mission()
  lap_test_once()
```

CAN生命周期：

```text
firmware/mspm0/wit-oled-hardware-spi/
Drivers/CAN/hball_can_port.[ch]
  hball_can_port_set_communication_enabled()
  hball_can_port_set_realtime_suspended()
```

选题客户端：

```text
firmware/mspm0/wit-oled-hardware-spi/
Drivers/CAN/hball_mission_client.[ch]
```

英飞凌Q3/Q4-Q6控制入口：

```text
firmware/edgetalk/rtthread/hball_bench_app.c
```

## 8. 下一线程必须采用的恢复与二分顺序

### 步骤A：先恢复硬件

1. 不修改源码。
2. 复位下烧录 `known-good-q2.hex`。
3. 确认 LCD 点亮。
4. 运行原始 `TASK1 FAST LAP`，确认仍能正常循迹。
5. 若原始 HEX 此时也不亮，先查供电、LCD排线和硬件，不进入软件二分。

### 步骤B：在独立 worktree 做硬件二分

不要 checkout 当前脏工作区。新建独立 worktree，依次构建烧录：

```text
babfb9e  -> 只验证LCD和SW3
a601b70  -> 只验证LCD和SW3
2735a09  -> 只验证LCD和SW3
7d1ee35  -> 只验证LCD和SW3
```

每次只记录：

```text
LCD是否点亮
SW3是否切题
SW1是否响应
是否进入HardFault/卡在初始化
镜像commit和SHA-256
```

找到第一个导致 LCD 不亮的提交后，只分析该提交的差异，不跨提交继续修。

### 步骤C：重新合并Q2时的硬约束

1. 从已验证原始工程的Q2函数和底层配置出发。
2. Q2启动前后都不依赖M33 READY、epoch或CAN状态。
3. Q2运行时不调用任何分布式任务API。
4. 先验证Q2连续多轮，再接Q3。
5. 每次只引入一个边界变化并单独提交、烧录、实测。

### 步骤D：再验证Q3-Q6

```text
Q3: 选择后开启CAN；SW1启动英飞凌；MSPM0保持STBY低
Q4: 英飞凌RUNNING后MSPM0才开始A到B循迹
Q5: 英飞凌RUNNING后MSPM0才开始整圈循迹
Q6: 英飞凌RUNNING后MSPM0才开始整圈循迹
```

Q3已调好的PID和状态机基线见：

```text
docs/ai-handoffs/q3-next-thread-handoff-20260731.md
docs/ai-handoffs/q3-state-machine-debug-retrospective-20260731.md
```

不要为修Q2修改Q3 PID；也不要为Q4 IMU补偿修改Q3静态控制律。

## 9. 构建和烧录

TI构建：

```bat
cd C:\Users\ASUS\Desktop\diansai\2026_TI\
firmware\mspm0\wit-oled-hardware-spi
build-keil.bat
```

生成：

```text
Keil\Objects\wit-oled-hardware-spi.axf
Keil\Objects\wit-oled-hardware-spi.hex
```

普通连接偶尔会出现：

```text
C Error: [__main__]
```

此时不要反复改代码，使用：

```text
-O connect_mode=under-reset -O reset_type=hw
```

## 10. 交接结论

当前最重要的事实只有三个：

1. 用户提供的 2026-07-30 01:56 原始 HEX 已实机证明 Q2 正常。
2. 重新集成状态机后的源码尚未得到可用Q2，当前HEAD甚至出现LCD不亮。
3. 正确架构由用户最终确认：Q2完全本地，Q3才开始通信，Q4-Q6为循迹加摆球。

下一线程的目标不是继续“调参”，而是先用硬件二分找到集成回归点，再从已验证
原始固件最小化移植选题与通信边界。

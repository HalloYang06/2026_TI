# Q3 钢球控制下一线程交接（2026-07-31）

## 1. 接手时先读

目标仓库是 `HalloYang06/2026_TI`，本机路径：

```text
C:\Users\ASUS\Desktop\diansai\2026_TI
```

当前分支 `prep/2026`，HEAD 为 `20b05e7`。工作区有未提交修改，禁止直接丢弃。
当前修改同时涉及 EdgeTalk M33、MSPM0、共享任务协议和本文档。

本题 Q3 的要求是：小车静止，钢球从 O 点运行到 `+50 mm`，折返到
`-50 mm` 并稳定，整个过程不超过 5 s，两个端点最大误差绝对值不超过
10 mm。

坐标约定：树莓派视觉 O 点为 0，钢球向合页轴心运动为负方向。

## 2. 上午真正成功的控制基线

不要把后续临时摩擦补偿当作基线。上午实机成功版本为：

```text
算法：PID
Kp = 0.70
Ki = 0.15
Kd = 0.40
静摩擦 boost = 0 mrad
Q3 IMU 前馈 = 禁用
RS00 水平标定 = 1.7205 rad（日志通常显示 1720/1721 mrad）
管角限制 = ±0.05236 rad（±3°）
```

上午的阶段判定：

```text
+50 mm：|位置误差| <= 10 mm 且 |估计速度| <= 20 mm/s，
         连续满足 150 ms 后才折返。
-50 mm：同样的位置/速度条件，连续满足 300 ms 后 PASS。
```

已经记录的实机结果：

- 原始 FinSH 基线使用 Kd=0.35：`0 -> +50 -> -50 mm`，4.024 s，
  最终约 `-45 mm`，
  `PASS`。
- 接入状态机后曾有一轮：到达 `+50 mm` 后折返，最终约 `-51 mm`，
  误差 1 mm，`passed=1`。

历史复盘文件：

```text
docs/ai-handoffs/q3-state-machine-debug-retrospective-20260731.md
```

## 3. 后来改坏过的内容

为处理重复测试时停在 `-32~-37 mm`，曾临时做过以下修改：

- 把运动阶段静摩擦补偿改为 2 mrad；
- 把保持阶段补偿改为 5 mrad；
- 把 `+50 mm` 的稳定时间从 150 ms 放宽到 50 ms；
- 取消 `+50 mm` 阶段的速度限制。

这些修改不是上午基线。实机出现过：

- 5 mrad 全程补偿时，钢球到正端后无法可靠折返；
- 2 mrad 时能在 2.819 s 瞬间判定 `-50 mm PASS`，随后又回滑到
  `-34 mm`；
- 因此“瞬间 PASS”不等于之后稳定保持。

上述控制器改动现已在源码中撤回：当前源码重新使用零摩擦补偿、
150 ms 和速度判定。下一线程不要未经 A/B 对照再次改 PID 或摩擦补偿。

## 4. 状态机接入后发现的问题

### 4.1 START 时电机并不一定在水平位

曾出现 LCD 已显示 Q3 READY，但 RS00 实际约 1775 mrad，而水平标定是
1721 mrad。旧状态机按 SW1 后先保持当前位置，再突然切换控制器的
1721 mrad 基准，会给钢球注入初速度。

当前未提交源码已加入 Q3 START_PENDING 自动回平：

- 每次最大移动 20 mrad；
- 到 1720.5 mrad 的误差不超过 3 mrad；
- 速度不超过 80 mrad/s；
- 连续稳定 100 ms 后才清估计器、清积分并启动 Q3 的 5 s 计时；
- 1.5 s 未回平则受控终止。

这个状态机包装应保留，它不应该改变上午 PID 控制律。

### 4.2 Q3 通过后被通用 15 s 超时关闭

旧代码在 Q3 已通过后仍会命中通用 commission 超时，自动回水平，钢球随即
离开 `-50 mm`。当前源码已让 `mode=Q3 && phase=3 && passed=1` 持续闭环，
切换到其他题目时才退出。

### 4.3 RESET/epoch/READY 同步

已处理过的问题包括：

- TI 持续 RESET 导致 M33 永远进不了 READY；
- TI 与 M33 epoch 不同导致状态残留；
- START_PENDING 中 RS00 参数尚未刷新就立即 ABORT；
- Q3 失败后 LCD 仍长期显示 RUNNING；
- Q3 误叠加车体 IMU 前馈。

不要删除 `20b05e7` 中的任务同步修复。Q3 的 IMU 前馈必须继续禁用，
Q4～Q6 才考虑 IMU 加速度前馈。

## 5. PB21 手动回平：当前未验证功能

用户希望比赛操作流程改为：

1. SW3 选择 Q3，等待 READY；
2. 按 PB21，仅让 RS00 回到 1720 mrad 并保持；
3. 人工把钢球放到 O 点；
4. 按 SW1 正式启动 Q3。

当前未提交源码已做了第一版实现：

- 共享协议新增 `HBALL_MISSION_COMMAND_LEVEL = 5`；
- MSPM0 的 PB21 已经在 SysConfig 中配置为
  `START_KEY_BUTTON_PIN / GPIOB.21`，任务菜单现在将它解释为 LEVEL；
- TI 持续发送 LEVEL，直到 SW1 把命令改为 START；
- M33 收到 LEVEL 后准备/使能 RS00、回到 1720.5 mrad，并清除位置积分、
  前次误差、输出估计和 boost 状态；
- SW1 后仍会执行一次 START_PENDING 自动水平确认，然后才开始计时。

涉及文件：

```text
shared/protocol/hball_mission_can.[ch]
firmware/edgetalk/src/hball_mission_arbiter.c
firmware/edgetalk/rtthread/hball_bench_app.c
firmware/mspm0/wit-oled-hardware-spi/Drivers/CAN/hball_mission_client.[ch]
firmware/mspm0/wit-oled-hardware-spi/Drivers/CAN/hball_can_port.[ch]
firmware/mspm0/wit-oled-hardware-spi/Drivers/GRAY/key.[ch]
firmware/mspm0/wit-oled-hardware-spi/main.c
```

注意：PB21 功能已经编译并烧入 TI 和 M33，但因随后 PI USB/视觉掉线，尚未完成
实机按键闭环验证。它只能算候选实现，不能算已验收功能。接手后先检查协议和状态机，
再实测 PB21，发现问题应修功能，不要先改 PID。

## 6. 当前烧录状态

### EdgeTalk M33

当前 M33 已用下列关键环境构建成功：

```text
HBALL_INTEGRATED_SHADOW=1
RTT_EXEC_PATH=F:\RT-ThreadStudio\platform\env_released\env\tools\gnu_gcc\arm_gcc\mingw\bin
```

RT-Thread Studio 工程：

```text
F:\RT-ThreadStudio\workspace\Edgi_Talk_M33_Blink_LED
```

生成 `build\rtthread.hex`，并通过
`tools\flash_m33_verified.ps1` 完成 raw verify 和 XIP verify。

### MSPM0G3507

工程：

```text
firmware\mspm0\wit-oled-hardware-spi
```

`build-keil.bat` 构建成功，生成：

```text
Keil\Objects\wit-oled-hardware-spi.hex
```

使用 Horco CMSIS-DAP（probe UID `2d2670f3`）和 pyOCD 烧录成功，随后显式
`reset`、`go`。

## 7. 当前阻塞：PI USB/视觉掉线

最后一次状态：

```text
hball_usb_status:
state=0x18
configured=0
conn=1
cfg=0
open=0
vision_rx=0
```

含义：EdgeTalk 检测到物理/VBUS 连接，但树莓派主机没有完成 USB reset/config，
不是 PID 故障。任务 ready mask 因缺少 PI_USB 和 VISION 无法 READY。

树莓派旧地址 `192.168.3.33` 已不在线；扫描 `192.168.3.0/24` 也未找到可 SSH 的
树莓派。用户已给树莓派重新上电，但仍未恢复。下一线程第一步应确认：

- 树莓派是否真正启动完成；
- 数据线是否接在 USB Host 数据口而非供电口；
- Linux 下是否出现 `/dev/ttyACM*` 或 `/dev/serial/by-id/*`；
- `hball-edgetalk-camera` 用户服务是否运行；
- 服务日志是否因设备路径变化退出。

在 `configured=1`、视觉恢复约 100 Hz 以前，不要进行 Q3 参数结论。

## 8. 下一线程建议执行顺序

1. 先保存当前未提交 diff，不要 reset 或 checkout。
2. 恢复 PI USB，确认 M33 `configured=1`、视觉约 100 Hz。
3. 不放球，选择 Q3 后按 PB21，验证 RS00 从任意小角度平滑回到
   `1720±3 mrad`，且保持不动。
4. 放球到 O，按 SW1，记录完整一轮 100 Hz 视觉和控制日志。
5. 连续做至少 5 轮，每轮都记录：
   - START 时电机角度和球位置；
   - 到 +50 mm 的时间、峰值和折返时速度；
   - 到 -50 mm 的时间；
   - 到达后 5 s、10 s 的保持位置；
   - `passed/phase/reason`。
6. 若上午控制器在相同起始条件下仍复现不了，优先比较启动基准、视觉零点、
   RS00 实际水平角和机构回差；不要先加摩擦 boost。
7. 验证 PB21 和至少 5 轮 Q3 后，再提交并推送。

## 9. 关键结论

目前最可信的控制结论只有：

```text
PID 0.70 / 0.15 / 0.40（在原始 0.35 基线上小幅增加阻尼后实机确认可用）
静摩擦补偿 0
Q3 禁用 IMU
+50 mm 严格速度判定并稳定 150 ms
-50 mm 稳定 300 ms
```

状态机应负责“同步、准备、自动回平、清状态、启动和持续保持”，不应悄悄改动
这套已经实机成功过的控制律。

# App / Mission 当前流程

## Happy path

```mermaid
flowchart TD
    Init["初始化硬件、通信、显示和电机<br/>main.c:79-102"] --> Loop{"start == 0?<br/>main.c:104-106"}
    Loop -- Yes --> Keys["轮询并阻塞消抖三个按键<br/>key.c:3-37"]
    Keys --> Round["选择圈数 0..5<br/>main.c:110-121"]
    Keys --> Question["选择题号 0..3<br/>main.c:123-133"]
    Keys --> Start["start = 1<br/>main.c:135-140"]
    Round --> Loop
    Question --> Loop
    Start --> Dispatch{"start != 0 且题号为 0?<br/>main.c:142-145"}
    Dispatch -- Yes --> Track["调用 my_track<br/>track.c:173-311"]
    Track --> Complete{"达到目标圈数?<br/>track.c:306-310"}
    Complete -- Yes --> Stop["写零 PWM 并 start = 0<br/>track.c:306-310"]
    Complete -- No --> Loop
    Stop --> Loop
    Dispatch -- No --> Stuck["无任务、无按键处理<br/>main.c:142-146"]
    Timer["20 ms ISR 无条件写电机<br/>main.c:151-170"] -.绕过任务状态.-> Track
    Timer -.可能覆盖停车.-> Stop
```

## 边界问题

- `main.c:79-147` 同时负责初始化、UI、任务选择和调度。
- `track.c:306-310` 反向修改 App 拥有的 `start`。
- `round_number` 默认值为 0；首次运行可以立即满足结束条件。
- 视觉题被选中后没有实际任务分支，主循环进入空转状态。
- TIMER0 控制路径完全绕过 Mission 状态。

## 外部依赖

- 输入：按键 GPIO、循迹状态、编码器、WIT yaw。
- 输出：LCD、蜂鸣器、LED、电机 PWM。
- 共享状态：`start`、`round_number`、`quetion_num`、三个 PID 单例。

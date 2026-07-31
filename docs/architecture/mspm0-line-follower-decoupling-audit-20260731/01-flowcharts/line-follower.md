# LineFollower 当前流程

```mermaid
flowchart TD
    Call["main 调用 my_track<br/>main.c:142-145"] --> Tick["圈数计算并递减调用计数器<br/>track.c:173-180"]
    Tick --> Forward{"turn_state == 1?<br/>track.c:182-194"}
    Forward -- Yes --> Drive["直行 1000 次调用后转弯<br/>track.c:183-194"]
    Forward -- No --> Turning{"turn_state == 2?<br/>track.c:195-207"}
    Turning -- Yes --> Pivot["原地左转 3000 次调用<br/>track.c:195-207"]
    Turning -- No --> Marker{"PIN0 与 PIN3 为高?<br/>track.c:209-212"}
    Marker -- Yes --> Arm{"标记已解锁?<br/>track.c:216"}
    Arm -- Yes --> Event["阻塞蜂鸣/闪灯、圈数加一<br/>track.c:218-227"]
    Event --> Drive
    Arm -- No --> Finish
    Marker -- No --> Lost{"PIN0-PIN6 全低?<br/>track.c:242-250"}
    Lost -- Yes --> Search["固定原地左转<br/>track.c:251-261"]
    Lost -- No --> Select["按 PIN1-PIN6 优先级选 PWM<br/>track.c:272-303"]
    Search --> Finish{"达到目标圈数?<br/>track.c:306-310"}
    Select --> Finish
    Finish -- Yes --> Stop["写零 PWM 并清 start<br/>track.c:306-310"]
    Finish -- No --> Call
```

## 边界问题

- 一个函数同时承担 GPIO 采样、状态机、速度决策、反馈输出和任务结束。
- 状态为十个非 `static` 文件级全局量，缺少 `init/reset/start/stop`。
- 所有时间均以调用次数表达，且会被阻塞式蜂鸣、闪灯改变。
- PIN7 未参与丢线判断；部分传感器组合不会产生新命令，会保持旧 PWM。
- 丢线条件的两个分支行为完全相同，属于无效分支。
- 直接依赖 `main.h`、应用全局量、SysConfig GPIO 宏与 Motor HAL。

## 外部依赖

- `DL_GPIO_readPins()` 与 `track_PIN_n_*`。
- `motor_pwm_set()`、`beep()`、`flash()`。
- `start`、`round_number` 和主循环调用频率。

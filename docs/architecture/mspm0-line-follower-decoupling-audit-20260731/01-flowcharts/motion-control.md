# MotionControl 当前流程

```mermaid
flowchart TD
    Timer["TIMER0 每 20 ms<br/>wit-oled-hardware-spi.syscfg:241-249"] --> Delta["计算编码器增量<br/>main.c:151-161"]
    Encoder["编码器 GPIO ISR 累加全局计数<br/>encoder.c:9-65"] --> Delta
    Wit["WIT UART/DMA ISR 更新 yaw<br/>interrupt.c:63-114"] --> Actual["写 LEFT/RIGHT/ANGLE.Actual<br/>main.c:162-164"]
    Delta --> Actual
    Actual --> Update["三次 PID_Update<br/>main.c:165-167"]
    Update --> Formula["误差、积分、死区和限幅<br/>pid.c:32-71"]
    Formula --> Mix["左右速度与航向混控<br/>main.c:168-169"]
    Mix --> Motor["motor_pwm_set<br/>main.c:170 / motor.c:69-100"]
    Motor --> GPIO["方向 GPIO<br/>motor.c:77-98"]
    Motor --> PWM["PWM 比较寄存器<br/>motor.c:20-31"]
    Track["前台循迹也直接写电机<br/>track.c:183-308"] -.竞争.-> Motor
    UART["UART ISR 改参数并调用 PID_Update<br/>uart_vofa.c:256-313"] -.并发修改.-> Update
```

## 边界问题

- 没有 MotionControl 的单一入口；ISR 同时负责采样、控制、混控和输出。
- 前台循迹与 20 ms ISR 是两个并发电机写入者。
- TIMER0 与 `start`、任务模式无关，上电即拥有 PWM 写权限。
- 编码器 A/B、LEFT/RIGHT 与第二电机取反的映射分散在三个文件。
- ANGLE 误差未处理 ±180° 环绕；PID 无积分抗饱和。
- UART ISR 与 TIMER ISR 并发修改同一 PID 对象。
- PID 算法层通过 `p == &ANGLE` 特判业务实例。

## 外部依赖

- 编码器累计全局量与 WIT 全局 `wit_data`。
- UART 调参协议。
- TIMA1 调度、TIMA0 PWM、方向 GPIO 和 STBY。
- 前台 LineFollower 的直接电机输出。

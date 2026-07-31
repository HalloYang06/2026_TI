# Platform / HAL 当前流程

```mermaid
flowchart TD
    Config["声明 GPIO/PWM/Timer/UART<br/>wit-oled-hardware-spi.syscfg:1-345"] --> Generated["生成全局硬件命名空间<br/>ti_msp_dl_config.h:50-341"]
    Generated --> Init["SYSCFG_DL_init<br/>ti_msp_dl_config.c:52-72"]
    Init --> Power["复位并上电外设<br/>ti_msp_dl_config.c:102-125"]
    Power --> GPIO["复用引脚并设置初值<br/>ti_msp_dl_config.c:128-277"]
    GPIO --> PWM["启动 1 kHz PWM、初始 50% compare<br/>ti_msp_dl_config.c:314-364"]
    PWM --> Timers["启动 20 ms / 10 ms 定时器<br/>ti_msp_dl_config.c:368-442"]
    Timers --> NVIC["应用层清除并启用 NVIC<br/>main.c:85-96"]
    NVIC --> MotorInit["motor_init 后由 main 拉高 STBY<br/>motor.c:5-18 / main.c:97-100"]
    Command["上层 motor_pwm_set<br/>motor.c:69-100"] --> Direction["立即切方向 GPIO<br/>motor.c:42-68"]
    Command --> Compare["写 PWM compare<br/>motor.c:20-33"]
    TickInit["SysTick_Init<br/>clock.c:22-26"] --> TickISR["tick_ms++<br/>interrupt.c:23-26"]
    TickISR --> TimeAPI["读取/忙等毫秒时间<br/>clock.c:7-20"]
    Missing["当前 main 未调用 SysTick_Init<br/>main.c:79-102"] -.导致 tick 不增长.-> TimeAPI
```

## 边界问题

- `SYSCFG_DL_init()` 同时配置、启动 PWM 和定时器，安全启动顺序依赖隐含 GPIO 初值。
- 生成头把寄存器实例、引脚和 IRQ 全部泄漏给业务模块。
- `motor.h` 暴露方向、单通道 PWM 和 GPIO 宏，无法保证命令一致提交。
- Timer、Encoder 和 UART 的 ISR 所有权分散在 `main.c`、`encoder.c` 和 `interrupt.c`。
- `interrupt.c` 理解多个传感器协议，平台层反向依赖设备层。
- SysTick API 已被多驱动依赖，但启动路径没有初始化它。

## 外部依赖

- TI MSPM0 SDK、SysConfig、DriverLib 和 CMSIS。
- App 对 NVIC、STBY 的直接操作。
- MotionControl、LineFollower 对 Motor HAL 的并发调用。
- Encoder/WIT/BNO 等模块提供的全局缓冲和数据结构。

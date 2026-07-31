# 分系统实施计划提示

以下提示可依次交给 `/make-plan`。顺序很重要：先统一电机所有权，再拆策略和传感器。

## 1. Motor HAL 与固定控制 tick

```text
/make-plan

为 wit-oled-hardware-spi 规划“单一电机写入所有权”改造。

目标组件：
- Platform/motor_hal.c，唯一入口 motor_hal_apply(WheelCommand)
- Platform/scheduler.c，TIMER0 ISR 只发布 20 ms control_due
- Control/motion_control.c，单一入口 motion_control_step(...)

必须重写的调用点：
- Drivers/GRAY/track.c:183-308 的全部 motor_pwm_set()
- main.c:151-170 的 TIMER0 PID/混控/电机输出
- main.c:97-100 的 motor/STBY 初始化
- Drivers/MOTOR/motor.h:15-21 的底层公开接口
- Drivers/MOTOR/motor.c:69-98 的左右重复提交逻辑

参考当前流程：
- PATHFINDER-2026-07-31/01-flowcharts/motion-control.md
- PATHFINDER-2026-07-31/01-flowcharts/platform-hal.md

成功标准：
- 全仓库只有 motor_hal.c 能直接写 AIN/BIN、PWM、STBY
- IDLE/FAULT 必定提交 STOP
- TIMER0 ISR 不运行 PID、不读取传感器、不写电机
- 架空轮测试验证左右轮正方向、STOP 和反转

反模式约束：
- 不增加 motor registry、factory 或运行时插件
- 不保留旧/新两条电机路径的 feature flag
- 不顺便重写循迹算法或 PID 参数
```

## 2. LineFollower 纯状态机

```text
/make-plan

为 wit-oled-hardware-spi 规划纯 LineFollower 状态机。

目标组件：
- Control/line_follower.c
- 单一入口 line_follower_step(LineFollower*, LineSample, now_ms)
- 输出 MotionIntent 和 FollowerEvent，不产生硬件副作用

必须重写的调用点：
- Drivers/GRAY/track.c:162-171 的文件级状态
- track.c:177-203 的调用次数计时
- track.c:209-303 的重复 GPIO 读取和 PWM 分支
- track.c:218-219 的 beep/flash
- track.c:306-309 的 start 修改
- track.c:7-160 的两代注释历史实现

参考当前流程：
- PATHFINDER-2026-07-31/01-flowcharts/line-follower.md

成功标准：
- 给定 bit mask 与 now_ms 可在主机单测中重现直行、左右修正、丢线、标记、完成
- 所有状态可 reset，重新开始不继承上次运行
- line_follower.c 不包含 ti_msp_dl_config.h、main.h、motor.h
- 状态 deadline 不依赖主循环调用次数

反模式约束：
- 不创建状态工厂或回调注册表
- 不保留三代算法可切换
- 不在这一步调整稳定版速度参数，先保持行为等价
```

## 3. Line / Encoder / WIT snapshots

```text
/make-plan

为 wit-oled-hardware-spi 规划三个明确的传感器快照边界。

目标入口：
- line_sensor_sample(now_ms)
- encoder_take_sample(now_ms)
- wit_get_sample(now_ms)

必须重写的调用点：
- track.c:209-303 的直接 GPIO 读取
- Drivers/ENCODER/encoder.c:3,9-64 对 main.c 全局计数的写入
- main.c:151-164 的非原子编码器差分
- Drivers/MSPM0/interrupt.c:63-114 的 WIT DMA/解析
- Drivers/WIT/wit.c:7-14 的 DMA 初始化
- Drivers/GRAY/gray.c:38-143 的无调用者软件 I2C 路径

参考当前流程：
- PATHFINDER-2026-07-31/01-flowcharts/sensors.md

成功标准：
- 三类 sample 都包含 value、valid、timestamp
- line 每个控制 tick 只采一次
- encoder 左/右与前进正方向在驱动边界归一化
- WIT 能区分“真实 0 度”和“从未收到有效帧”
- ISR 只修改各自模块私有数据

反模式约束：
- 不做通用 Sensor 基类、factory 或 registry
- 不强行合并 WIT 与 BNO parser
- 未证明存在第二硬件 SKU 时，不为死掉的软件 I2C 路径保留 backend 框架
```

## 4. Mission 与非阻塞反馈

```text
/make-plan

为 wit-oled-hardware-spi 规划 MissionController 和非阻塞反馈。

目标组件：
- App/mission.c，单一入口 mission_step(...)
- App/feedback.c，单一入口 feedback_step(now_ms)
- 显式 IDLE/TRACKING/STOPPING/FAULT 状态

必须重写的调用点：
- main.c:63-66,104-145 的 start/round_number/quetion_num
- track.c:306-309 的反向停止
- Drivers/GRAY/key.c:6-35 的三份阻塞消抖
- Drivers/GRAY/beeper.c:3-8 与 led.c:4-8 的阻塞反馈
- main.c:123-133 的无消费者视觉题选择

参考当前流程：
- PATHFINDER-2026-07-31/01-flowcharts/app-mission.md

成功标准：
- 只有 Mission 改任务状态
- 任意状态都有明确 STOP/FAULT 回退
- LineFollower 完成通过事件上报
- 按键、蜂鸣和 LED 不阻塞 20 ms 控制 tick
- main.c 只负责初始化、取事件和连接模块

反模式约束：
- 不做事件总线或任务注册表
- 不保留没有实现的视觉模式占位状态
- 不让 Mission 直接写 GPIO/PWM
```

## 5. PID 与 UART 调参边界

```text
/make-plan

为 wit-oled-hardware-spi 规划 PID 和 UART 调参解耦。

目标组件：
- Control/pid.c，单一入口 pid_step(pid,setpoint,measurement,dt)
- MotionControl 私有拥有 LEFT/RIGHT/ANGLE 控制器
- UART 只产生 PidConfigUpdate，在下一 control tick 应用

必须重写的调用点：
- Drivers/PID/pid.c:5-26 的全局单例
- pid.c:32-73 的 ANGLE 地址特判与积分实现
- pid.c:80-111 的注释旧实现
- main.c:162-170 的 PID 执行
- Drivers/UART_VOFA+/uart_vofa.c:256-339 的 ISR 参数修改和 PID_Update
- uart_vofa.c:324-352 的两套接收缓冲

参考当前流程：
- PATHFINDER-2026-07-31/01-flowcharts/motion-control.md

成功标准：
- PID 只有 control tick 一个执行入口
- UART ISR 不修改 PID 历史状态、不调用 pid_step
- angle wrap 在调用边界显式处理
- 有 anti-windup，PID 与 Motor 两级限幅均保留
- 调参协议有长度边界和完整帧语义

反模式约束：
- 不建立 PID 字段反射表或通用命令 registry
- 不保留 ISR 与主循环两种执行模式
- 不在此步骤改变已经验证过的控制参数
```

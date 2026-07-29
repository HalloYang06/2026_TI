# 天猛星 MSPM0G3507 小车循迹固件

这是 2026 年 TI 电子设计竞赛 H 题准备阶段的小车底盘固件。当前版本完成了
LCD、WIT IMU、电机、单相编码器、8 路数字灰度循迹和按键跑圈测试。

## 当前控制方案

- 8 路灰度传感器采用加权中心位置：
  `{-35, -25, -15, -5, 5, 15, 25, 35}`。
- 循迹外环采用比例差速：
  `steering = 0.50 * weighted_error`。
- 左右目标速度以每次最多 2 counts/100 ms 的速度平滑变化。
- 左右轮使用独立 PI 速度闭环，控制周期为 100 ms：
  `Kp = 0.18`，`Ki = 0.005`。
- 丢线时按最后检测到的方向低速搜索，700 ms 未恢复则停车。
- 天猛星板载 PB21 按键用于启动跑圈；上电后不会自动驱动电机。

## 已确认引脚

| 功能 | MCU 引脚 |
|---|---|
| CAN TX | PA26 |
| CAN RX | PA27 |
| 灰度 S1 | PB17 |
| 灰度 S2 | PB18 |
| 灰度 S3 | PB24 |
| 灰度 S4 | PB25 |
| 灰度 S5 | PA22 |
| 灰度 S6 | PB20 |
| 灰度 S7 | PA23 |
| 灰度 S8 | PB22 |
| 启动按键 | PB21 |
| 编码器 E1A | PA12 |
| 编码器 E2A | PB12 |

PB13 对应的编码器 B 相在实板上异常，因此当前左右轮均使用单相计数。实测每轮
约 400 counts/rev。电机输出 1 对应 E1A/countB，电机输出 2 对应
E2A/countA。

## 构建与烧录

依赖 MSPM0 SDK 和 Keil ArmClang。命令行使用方法见
[README-command-line.md](README-command-line.md)。

```bat
build-keil.bat
flash-daplink.bat
```

`wit-oled-hardware-spi.syscfg` 是引脚配置源文件；当前生成文件保存在
`Debug/ti_msp_dl_config.c` 和 `Debug/ti_msp_dl_config.h`。

## 实机验证

- LCD 和 WIT 实时角度显示正常。
- 两个电机方向、编码器计数和 8 路灰度输入正常。
- PWM 20%～40% 分档测速呈近似线性。
- 双轮 PI 悬空测试可稳定在约 60 counts/100 ms。
- 加权位置比例差速版本已完成实地跑圈测试，主观效果良好。

硬件测试时应保证车辆可立即提起或断电，首次修改电机方向、PWM 或循迹参数时应先
悬空验证。

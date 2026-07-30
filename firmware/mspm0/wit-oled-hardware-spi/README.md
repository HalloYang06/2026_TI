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

## H 题 CAN 遥测

- 总线固定为 `1 Mbps Classic CAN`，PA26=`CANFD0_CANTX`、
  PA27=`CANFD0_CANRX`；必须经过 3.3 V CAN 收发器，MCU 引脚不能直接接
  CANH/CANL。
- 40 MHz `SYSPLLCLK1` 作为 MCAN 时钟。SysConfig 生成的标称位时序为
  `NBRP=1, NTSEG1=34, NTSEG2=5, NSJW=5`，采样点 `87.5%`。
- MSPM0 发布 `0x080/0x100/0x101/0x102/0x103` 五类 8 字节标准帧，
  频率分别为 `20/200/200/100/100 Hz`；每类使用独立 16 位序号。
- RX FIFO0 开放接收标准帧和 29 位扩展帧，因此可在调试器中通过
  `g_hball_can_stats` 核对 EdgeTalk 诊断帧、RS00反馈、TEC/REC、bus-off、
  FIFO丢失和最后一帧。
- 安全默认值保持 `HBALL_CAN_MOTOR_COMMAND_TX_ENABLED=0`。尚未映射硬件急停，
  因此心跳始终置 `ESTOP_ACTIVE`，且不置 `CHASSIS_READY`；轮周长未实测前
  `0x102` 米制轮速保持为零。CAN 联通不代表允许运动。

物理总线接成短支线的线型拓扑：MSPM0、EdgeTalk 和 RS00 共地、CANH 对
CANH、CANL 对 CANL，总线两端各 120 Ω；断电测 CANH-CANL 应约为 60 Ω。
首轮只允许观察遥测和人工发送 RS00 `Get_ID`，禁止 enable、set-zero、位置、
速度和力矩命令。

实现依据为 TI 官方 [MSPM0 SDK](https://www.ti.com/tool/MSPM0-SDK)
`2.05.01.00`、[MSPM0G1x0x/G3x0x MCAN DriverLib API](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/2_05_01_00/docs/english/driverlib/mspm0g1x0x_g3x0x_api_guide/html/group___m_c_a_n.html)
和 SDK 内的 `mcan_single_message_tx`、`mcan_message_rx` 官方示例。参考仓库的
机械臂业务、零点和电机参数没有复制到本工程。

## 构建与烧录

依赖 MSPM0 SDK 和 Keil ArmClang。当前 SysConfig 源和生成文件均以
`mspm0_sdk_2_05_01_00` 验证。命令行使用方法见
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

# EdgeTalk 滚球控制固件

## 当前状态

M33 已有两个显式构建模式：默认的只读 CAN 台架，以及 `HBALL_USB_ONLY=1` 的 USB CDC 联调镜像。USB-only 模式只编入 H-ball 的 `main.c`、`hball_usb_cdc.c` 和 `hball_usb_probe.c`，不编入 H-ball CAN 适配器；P16.5 蓝灯每 500 ms 翻转，用来区分“未启动”和“USB 未枚举”。

当前实现已按 Infineon 官方 PSoC Edge CDC echo 启动顺序切换到 BSP 自带 emUSB 2.1.0.3859，并完成 M33 编译、Secure+NS 合并、烧录和逐字节校验。P16.5 蓝灯正常闪烁，Non-secure reset handler 可达；FinSH 显示 `state=0x11 configured=0 conn=1 cfg=0 actuator_tx=0`，即 emUSB 已 `ATTACHED|SUSPENDED`、尚未被树莓派配置。树莓派仍无 `lsusb` 插拔记录和 `/dev/ttyACM*`，所以 USB 通信尚未打通。

当前实验主机是树莓派，目标链路为 `USB Host -> EdgeTalk USB Device -> /dev/ttyACM*`。协议探针周期发送 `HBALL_USB_READY`，接收 `PING <seq> <payload>` 后回复 `PONG`；该文本探针只用于打通链路，正式视觉控制包仍按 ADR-002 的二进制帧设计。

## USB-only 构建约束

- 构建环境必须设置 `HBALL_USB_ONLY=1`。
- PSE84 BSP 必须开启 `CONFIG_BSP_USING_USB=y`，关闭 `RT_USING_CHERRYUSB`；双栈同时启用会竞争 USBHS 和中断。
- menuconfig 的通用 `Using USB -> Using USB host/device` 对应 RT-Thread 旧栈 `RT_USING_USB_HOST/RT_USING_USB_DEVICE`，不是 Infineon emUSB 开关；本方案两项都保持关闭。真正需要打开的是 `Board peripheral drivers -> Enable USB`，对应 `BSP_USING_USB`。
- `libraries/components/SConscript` 在 `BSP_USING_USB` 时必须加入 `emusb-device/SConscript`；emUSB 定时依赖还需编入 `mtb_hal_timer.c` 与 `cy_tcpwm_counter.c`。
- CDC 初始化顺序固定为 `USBD_Init()`、`hball_usb_add_cdc()`、`USBD_SetDeviceInfo()`、`USBD_Start()`，端点参数跟随 Infineon 官方 HS CDC 示例。
- SCons 的原始 `rtthread.hex` 只有 Non-secure XIP 段，不能直接烧录；必须按 BSP 的 `boot_with_extended_boot_scons.json` relocate 后与签名 Secure HEX 合并，再生成 XIP 校验镜像。
- 自动构建和测试不连接电机、不发送 CAN、不启动运动。实机烧录前必须断开底盘/电机动力，轮子或执行器卸载，仅使用调试器/USB 供电，并保留拔线断电接管。

验证命令：

```powershell
python -m pytest firmware/edgetalk/tests vision/raspberrypi/tests -q
```

当前主仓库结果以本次提交的测试输出为准；临时 BSP emUSB 静态契约为 `6 passed`，M33 构建大小为 `text=190096 data=15616 bss=244300`。

## 已验证的构建与烧录流程

在 RT-Thread Studio EdgeTalk BSP 1.1.0 的 M33 工程中，将本目录的 `SConscript`、`include/hball_usb_probe.h`、`src/hball_usb_probe.c`、`rtthread/main.c` 和 `rtthread/hball_usb_cdc.c` 同步到 `applications/hball/`，并让 `applications/SConscript` 包含该目录。然后执行：

```powershell
$env:HBALL_USB_ONLY='1'
scons -j12
python tools/test_pse84_official_emusb_cdc_static.py
```

`SConstruct` 使用 `config/boot_with_extended_boot_scons.json` 将 NS HEX 从 XIP 别名 relocate 到 raw flash 地址，并与 `tools/edgeprotecttools/cm33_s_signed_fw/proj_cm33_s_signed.hex` 合并为 `build/rtthread.hex`。烧录前必须先由 OpenOCD `flash banks` 确认 `cat1d.cm33.smif1_ns` 位于 `0x60000000`，再执行写入、raw verify、XIP verify，不能只看进程退出码。

本次已验证产物：raw combined SHA-256 `6FF1D97D0A833B490BD0D33FF5E615D6C66ED98522668940C49C5D13C379ACAB`；XIP verify `65EF82513764233A98BF10184F13298E61A9ED3840B2AE621A3AFD532CFA3364`；NS `453A13997A17E347EA5D5025170E247F4E35D4C79C69FEE991389E6922B2ED71`。实测写入并校验 raw `315392` bytes、XIP `308168` bytes、NS `205712` bytes。

## 下一台电脑的首轮检查

已确认使用的数据线可传输数据，不再把线材本身列为首要嫌疑。保持电机动力断开、轮子/执行器卸载、仅调试器/USB 供电和可拔线急停，按以下顺序继续：

1. 确认树莓派使用具备 Host 功能的 USB-A 口，EdgeTalk 使用板上标“USB”的 Device Type-C 口。
2. 同时观察树莓派 `sudo dmesg -w`、`watch -n 0.5 lsusb` 和 EdgeTalk `hball_usb_status`。
3. 复测 EdgeTalk 官方 `P17.4 VBUS_DETECT`；本次配置为输入后读低，DWC2 `DCTL=0` 且从未收到主机 USB reset，重点检查板级 VBUS 检测/供电路径与端口角色。
4. 只有出现 `/dev/ttyACM*` 后才运行 `python3 vision/raspberrypi/edgetalk_usb_probe.py --duration 60 --rate 20`；未枚举时不要叠加 CAN 或控制算法。

官方依据：

- [Infineon PSoC Edge USB CDC echo 示例](https://github.com/Infineon/mtb-example-psoc-edge-usb-device-cdc-echo/tree/42fbdaeeac61c8b9eae049855b488f0862d9385c)
- [Infineon emUSB Device 2.1 文档](https://infineon.github.io/emusb-device/html/index.html)

## 计划职责

- 200 Hz：读取带采集时间戳的 MSPM0状态和树莓派视觉测量，运行非线性预测、Kalman更新、5帧球速估计与LQR。
- 1 kHz：摆杆角度/速度内环、目标角 `+-4 deg` 限幅、`80 deg/s` 斜率限制和执行器抗饱和。
- 电机接口：读取角度、速度、温度、故障、电流/限幅状态；发送经过安全门批准的目标。
- 20 kHz FOC：仅在确认 EdgeTalk 直接驱动三相桥时实现。若RS00内部驱动器已闭合FOC，则使用其协议接口，不在M33重复闭环。
- 安全状态机：视觉/IMU/编码器超时、电机故障、母线压降、通信CRC/序号异常和人工急停。

## 实现顺序

1. 先实现与硬件无关的定点/浮点控制内核，并用 `experiments/h_ball_control_sim` 的golden trace对拍。
2. 实现只读通信和时间同步，自动测试只允许回放数据。
3. 接入只读电机 Get_ID与反馈，验证fresh/stale和单位换算。
4. 用实测相机、机构和电机参数重新生成增益和压力报告。
5. 经人工台架检查后再增加手动使能路径；自动测试永久禁止使能。

参考仓库与工具链记录见 `docs/reference/infineon-edgetalk-motor5.md`。

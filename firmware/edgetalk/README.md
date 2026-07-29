# EdgeTalk 滚球控制固件

## 当前状态

M33 有三个显式构建模式：默认只读 CAN 台架、`HBALL_USB_ONLY=1` 的 USB CDC 联调镜像，以及 `HBALL_INTEGRATED_SHADOW=1` 的 USB+CAN+200 Hz传感器快照镜像。集成模式仍不连接M55 IPC和任何执行器发送；P16.5蓝灯每500 ms翻转，用来区分“未启动”和“USB未枚举”。

当前实现已按 Infineon 官方 PSoC Edge CDC echo 启动顺序切换到 BSP 自带 emUSB 2.1.0.3859，并完成 M33 编译、Secure+NS 合并、烧录、逐字节校验和树莓派双向压力测试。P16.5 蓝灯正常闪烁，FinSH 为 `state=0x1e configured=1 conn=1 cfg=1 actuator_tx=0`；树莓派枚举为 `058b:0282`、`cdc_acm`、High-Speed 480 Mbps，并生成 `/dev/ttyACM0` 和稳定的 `/dev/serial/by-id/...HBALL-PROBE-if00`。二进制接收每次最多读取官方 HS Bulk 的 512 字节，再交给跨读取流解析器，不把一次 CDC Receive 当成一帧。

当前实验主机是树莓派，目标链路为 `USB Host -> EdgeTalk USB Device -> /dev/ttyACM*`。emUSB 接收必须像官方示例一样直接阻塞调用 `USBD_CDC_Receive(..., 0)`，不能先查 `USBD_CDC_GetNumBytesInBuffer()`；后者会造成 OUT 端点从未提交接收请求。设备在进入阻塞接收前发送 `HBALL_USB_READY`，接收 `PING <seq> <payload>` 后回复 `PONG`。主机打开串口并切换 raw 模式后先发送一个换行分隔符，再等待新 READY，以清除 Linux TTY 初始回显残片。该文本探针只用于打通链路，正式视觉控制包仍按后续二进制协议设计。

## USB-only 构建约束

- 构建环境必须设置 `HBALL_USB_ONLY=1`。
- PSE84 BSP 必须开启 `CONFIG_BSP_USING_USB=y`，关闭 `RT_USING_CHERRYUSB`；双栈同时启用会竞争 USBHS 和中断。
- menuconfig 的通用 `Using USB -> Using USB host/device` 对应 RT-Thread 旧栈 `RT_USING_USB_HOST/RT_USING_USB_DEVICE`，不是 Infineon emUSB 开关；本方案两项都保持关闭。真正需要打开的是 `Board peripheral drivers -> Enable USB`，对应 `BSP_USING_USB`。
- `libraries/components/SConscript` 在 `BSP_USING_USB` 时必须加入 `emusb-device/SConscript`；emUSB 定时依赖还需编入 `mtb_hal_timer.c` 与 `cy_tcpwm_counter.c`。
- CDC 初始化顺序固定为 `USBD_Init()`、`hball_usb_add_cdc()`、`USBD_SetDeviceInfo()`、`USBD_Start()`，端点参数跟随 Infineon 官方 HS CDC 示例。
- SCons 的原始 `rtthread.hex` 只有 Non-secure XIP 段，不能直接烧录；必须按 BSP 的 `boot_with_extended_boot_scons.json` relocate 后与签名 Secure HEX 合并，再生成 XIP 校验镜像。
- 自动构建和测试不连接电机、不发送 CAN、不启动运动。实机烧录前必须断开底盘/电机动力，轮子或执行器卸载，仅使用调试器/USB 供电，并保留拔线断电接管。

## 集成 shadow 构建

集成模式需要同时开启 `BSP_USING_USB`、`RT_USING_CAN`、`BSP_USING_CAN`和`BSP_USING_CANFD0`，并执行：

```powershell
$env:HBALL_USB_ONLY='0'
$env:HBALL_INTEGRATED_SHADOW='1'
scons -j12
```

M33把五类MSPM0标准帧、RS00扩展反馈帧和树莓派视觉帧汇总为200 Hz快照。USB和CAN线程通过RT-Thread优先级继承mutex更新单写数据层；每类数据独立计算age和valid。当前快照保留视觉板原始`capture_time_us`，但在时钟同步实现前只把`vision_receive_age_ms`用于链路诊断，不能冒充真实采集age。

本机集成ARM构建已通过：`text=210100 data=15656 bss=244256`。只有人工执行的`hball_probe5`允许发送无运动Get_ID；自动探针默认关闭，`MOTOR_COMMAND_TX=0`和`ACTUATOR_TX=0`保持硬约束。

## M55算法与LVGL shadow构建

M55使用独立的`SConscript.m55`，只编入双核IPC、LQG、多速率控制管线、H题LVGL页面和M55入口，不链接M33 CAN监视器或机械臂应用。控制线程使用绝对周期唤醒，以5 ms周期读取M33传感器槽、运行200 Hz预测/更新并写回带`SHADOW_ONLY`的控制槽；LVGL以100 ms周期显示球位置/速度、IMU加速度、转弯角速度、电机角度、LQG shadow目标和视觉age。`ACTUATOR_TX=0`，M55没有CAN或执行器发送。

临时M55 BSP曾完成全量GCC 13.3链接并得到`text=478232 data=2936 bss=4394376`，但复核ELF属性后确认该临时工程实际使用`-mcpu=cortex-m7`并生成ARMv7E-M镜像；这只能证明H题应用源码可链接，不能作为Cortex-M55可烧录证据。正式部署必须改用厂商Cortex-M55启动、异常/FPU上下文和cache配置，核验最终ELF为ARMv8.1-M且链接地址位于M55区域。临时BSP和裁剪配置不进入Git；真实M55构建完成前不能烧录该镜像，也不能把shadow目标接到电机。

验证命令：

```powershell
python -m pytest firmware/edgetalk/tests vision/raspberrypi/tests -q
```

当前主仓库同时覆盖文本探针、64字节视觉帧、CRC32C、坏帧重同步和USB拆/粘包。二进制版本的本机 ARM 构建已通过，大小为 `text=191408 data=15616 bss=244300`；烧录与240 Hz实物接收计数仍需单独验证。

## 已验证的构建与烧录流程

在 RT-Thread Studio EdgeTalk BSP 1.1.0 的 M33 工程中，将本目录的 `SConscript`、`include/hball_usb_probe.h`、`include/hball_vision_protocol.h`、对应两个 `src/*.c`、`rtthread/main.c` 和 `rtthread/hball_usb_cdc.c` 同步到 `applications/hball/`，并让 `applications/SConscript` 包含该目录。然后执行：

```powershell
$env:HBALL_USB_ONLY='1'
scons -j12
python tools/test_pse84_official_emusb_cdc_static.py
```

`SConstruct` 使用 `config/boot_with_extended_boot_scons.json` 将 NS HEX 从 XIP 别名 relocate 到 raw flash 地址，并与 `tools/edgeprotecttools/cm33_s_signed_fw/proj_cm33_s_signed.hex` 合并为 `build/rtthread.hex`。烧录前必须先由 OpenOCD `flash banks` 确认 `cat1d.cm33.smif1_ns` 位于 `0x60000000`，再执行写入、raw verify、XIP verify，不能只看进程退出码。

最终已验证产物：raw combined SHA-256 `D5C8FB63A28A5405088AE803A080AF483BB23B3A87A94B190ABE55B2011D1A80`；XIP verify `BFE5092E229F9D9A2D4582FB0BE118B0B05F50948154E214FADEF76FE932774B`；NS `982AC90DCA53AAF21A2E0BBC5052778440AFFDBF092956DF55D028670A8E2434`。实测写入并校验 raw `315392` bytes、XIP `310144` bytes、NS `207688` bytes。

## 树莓派实测与复现

已确认使用的数据线可传输数据。保持电机动力断开、轮子/执行器卸载、仅调试器/USB 供电和可拔线急停，按以下顺序复现：

1. 确认树莓派使用具备 Host 功能的 USB-A 口，EdgeTalk 使用板上标“USB”的 Device Type-C 口。
2. 同时观察树莓派 `sudo dmesg -w`、`lsusb -t` 和 EdgeTalk `hball_usb_status`，验收 `configured=1` 与 `cdc_acm`。
3. 运行 `python3 vision/raspberrypi/edgetalk_usb_probe.py --duration 30 --rate 100 --timeout 0.10`。本次实测 `2960/2960`、零超时，平均 RTT `1.60 ms`、P95 `2.14 ms`。
4. 连续 5 次关闭/重开串口均通过，共 `597/597`；最终可复现镜像再次以 100 Hz 验证 `987/987`，平均 RTT `1.53 ms`、P95 `2.08 ms`。
5. 每次打开串口会发送一个空行做同步，因此 `invalid_rx` 会增加 1；正式 PING/PONG 的失败判据仍是 `timeout/unexpected/tx_fail/rx_fail` 非零。

二进制视觉接收在同一安全状态下运行：

```bash
python3 vision/raspberrypi/edgetalk_vision_stream.py --duration 30 --rate 240
```

发送完成后从独立 KitProg/FinSH 串口执行 `hball_usb_status`，验收 `vision_rx` 增量等于主机 `tx_frames`，且 `vision_crc=0`、`ooo=0`、`gap=0`。主机的 `HOST_PASS` 只证明写入成功，必须结合 M33 计数才算端到端通过。

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

# EdgeTalk 滚球控制固件

## 当前状态

M33 有三个显式构建模式：默认只读 CAN 台架、`HBALL_USB_ONLY=1` 的 USB CDC 联调镜像，以及 `HBALL_INTEGRATED_SHADOW=1` 的 USB+CAN+200 Hz传感器快照镜像。集成模式会发布M33->M55共享快照并读取M55 `SHADOW_ONLY`结果，但不包含任何执行器发送；P16.5蓝灯每500 ms翻转，用来区分“未启动”和“USB未枚举”。

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

M33把五类MSPM0标准帧、RS00扩展反馈帧和树莓派视觉帧汇总为200 Hz快照。USB和CAN线程通过RT-Thread优先级继承mutex更新单写数据层；每类数据独立计算age和valid。MSPM0各数据流现在独立检查16位源序号：重复/乱序帧不刷新时间戳，跳号和重启可诊断，心跳未置`IMU_VALID`时不发布有效IMU。MSPM0的200 Hz镜像重复同一WIT源序号时也不会刷新age。当前快照保留视觉板原始`capture_time_us`，但在时钟同步实现前只把`vision_receive_age_ms`用于链路诊断，不能冒充真实采集age。

本机集成ARM构建已通过：`text=214596 data=15656 bss=244516`，运行标签为`0.3.0-m33-integrated-shadow`。只有人工执行的`hball_probe5`允许发送无运动Get_ID；自动探针默认关闭，`MOTOR_COMMAND_TX=0`和`ACTUATOR_TX=0`保持硬约束。

2026-07-30已将该集成镜像烧入实板：OpenOCD写入并校验raw Secure+NS `339,968 bytes`，组合XIP校验`332,708 bytes`，NS校验`230,252 bytes`，随后到达Non-secure reset handler。树莓派CDC守护完成自动重连；人工执行一次只读Get_ID后，RS00回复有效，CAN的TEC/REC、pending、bus-off和FIFO丢失均为0。随后接入MSPM0，五类标准遥测均被有效解析；姿态提升到200 Hz后，MSPM0目标总率为720 frame/s，14 s只读日志实测约729.1 frame/s且`tx_fail=0`。这证明三节点物理链路和遥测合同已打通，不代表连续RS00角度反馈或运动闭环已经启用。

## M55算法与LVGL shadow构建

正式M55运行时改用Infineon官方FreeRTOS `release-v10.6.202`，固定提交`8a19c8db81becf1e981a5f94630952160fddf8c5`和`COMPONENT_CM55/TOOLCHAIN_GCC_ARM` port。`freertos/Makefile.hball.mk`只编入双核IPC、LQG、多速率控制管线、FreeRTOS任务和M55入口，不链接M33 CAN监视器或机械臂应用。控制任务使用`vTaskDelayUntil`绝对周期唤醒，以5 ms周期读取M33传感器槽、运行200 Hz预测/更新并写回带`SHADOW_ONLY`的控制槽；100 ms钩子向板级LVGL适配层提供球位置/速度、IMU加速度、转弯角速度、电机角度、LQG shadow目标和视觉age。`ACTUATOR_TX=0`，M55没有CAN或执行器发送。

`SConscript.m55`保留用于既有RT-Thread适配器的源码契约检查，不再作为正式M55部署依据。2026-07-30已在Infineon官方多核示例的临时工程中完成180/180源文件编译和单核链接，得到664,588字节ELF，SHA-256为`005A5BB383F6D6C57BC4FCF34E6D56B43D82B3CE0F5992B6A651529CCF96DD73`。ELF为ARMv8.1-M Mainline、hard-float，实际编译参数为`-mcpu=cortex-m55+nomve`，证明正式FreeRTOS/H-ball源码能在真CM55工具链链接；当前明确未启用MVE。

这个ELF仍不是可部署镜像。官方示例的新内存设计把`.hball_ipc_shared`放在`0x262FC000`，而当前已验证M33 RT-Thread镜像将同一段放在`0x261C0000`；两核地址不一致，不能启动IPC。多核后处理还因缺少匹配的`proj_cm33_ns.hex`而返回失败。下一步必须用当前M33的同一份Device Configurator内存设计重新生成CM55链接图，使两边都得到`start=0x261C0000/end=0x261C0100`，再生成完整多核包。完成前禁止烧录M55，也禁止把shadow目标接到电机。

验证命令：

```powershell
python -m pytest firmware/edgetalk/tests vision/raspberrypi/tests -q
```

当前主仓库同时覆盖文本探针、64字节视觉帧、CRC32C、坏帧重同步、USB拆/粘包，以及CAN帧经统一快照进入LQG shadow的端到端主机测试。集成M33 ARM构建已通过，大小为`text=214596 data=15656 bss=244516`；真实240 Hz二进制接收计数仍需单独验证。

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

- 当前200 Hz M55 shadow：读取MSPM0状态和100 Hz带时间戳视觉，运行旧LQG回归；后续按
  RS00两连杆基线替换为OOSM Kalman、LQI、IMU前馈和端部保护。
- 1 kHz：M33检查M55 shadow的freshness、有限值和故障状态；部署前把仿真的正常`+-5 deg`、
  恢复约`+-7 deg`、硬限位`+-8 deg`以及两连杆逆解加入安全门；当前只观察，不发送。
- 200 Hz电机目标：正式方案采用RS00 CSP位置模式，写入经安全门批准的`loc_ref`并配置保守`limit_spd/limit_cur`；当前`ACTUATOR_TX=0`，发送适配器尚未实现。
- 250~500 Hz电机反馈：读取角度、速度、温度、故障和跟随误差；具体周期以台架实测为准。
- 位置环与FOC：留在RS00内部驱动器，频率在取得厂家资料或实测前不假定，M33不重复实现三相电流环。MIT位置-速度阻抗模式仅作为CSP带宽不足时的备选。
- 安全状态机：视觉/IMU/编码器超时、电机故障、母线压降、通信CRC/序号异常和人工急停。

## 实现顺序

1. 先实现与硬件无关的定点/浮点控制内核，并用 `experiments/h_ball_control_sim` 的golden trace对拍。
2. 实现只读通信和时间同步，自动测试只允许回放数据。
3. 接入只读电机 Get_ID与反馈，验证fresh/stale和单位换算。
4. 用实测相机、机构和电机参数重新生成增益和压力报告。
5. 经人工台架检查后再增加手动使能路径；自动测试永久禁止使能。

参考仓库与工具链记录见 `docs/reference/infineon-edgetalk-motor5.md`。

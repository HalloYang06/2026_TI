# ADR-003：EdgeTalk M33 USB CDC 采用官方 emUSB + USB-only

## 状态

实验性采用；固件已烧录并运行，等待 Linux 枚举与 PING/PONG 验证

## 日期

2026-07-30

## 背景

树莓派通过 EdgeTalk 标有 USB 的 Type-C 口连接后，Linux 没有出现 `/dev/ttyACM*`。经过 Secure+NS 镜像合并、烧录和校验后，P16.5 蓝灯已正常闪烁，证明 Non-secure RT-Thread main 正在调度。需要继续把主机端口角色、VBUS、USB reset、枚举、CDC 协议和 CAN/执行器变量逐层隔离。

RT-Thread v5.2.2 本身没有 PSE84 BSP，完整 EdgeTalk 工程来自 RT-Thread Studio BSP tag `1.1.0`。CherryUSB 首轮虽能编译，但最终采用 BSP 自带 Infineon emUSB 2.1.0.3859，并严格跟随官方 PSoC Edge CDC echo 示例的启动顺序和 HS CDC 端点配置。

## 决策

- M33 使用 BSP 内 emUSB 2.1.0.3859，开启 `BSP_USING_USB` 并关闭 CherryUSB。
- RT-Thread 通用菜单 `Using USB -> Using USB host/device` 属于另一套旧 USB 栈，对应 `RT_USING_USB_HOST/RT_USING_USB_DEVICE`，两项均保持关闭；只打开板级 `Board peripheral drivers -> Enable USB`。
- `HBALL_USB_ONLY=1` 时 H-ball 应用只编译心跳 main、CDC 适配器和无硬件协议解析器；CAN 监视器、CAN 驱动应用和控制算法不进入镜像。
- 蓝色 P16.5 LED 每 500 ms 翻转。亮灭本身不代表枚举成功，只证明 Non-secure RT-Thread main 正在调度。
- CDC 启动顺序固定为 `USBD_Init()`、注册 CDC、设置设备信息、`USBD_Start()`；端点布局跟随官方 High Speed CDC echo。
- 禁止同时启用 emUSB 与 CherryUSB；两者会竞争 USBHS 和 IRQ。
- 烧录物必须包含 relocate 后的 Non-secure raw 段和签名 Secure 段。SCons 直接产生的 NS `rtthread.hex` 只能用于 XIP 校验，不能单独烧录。

## 备选方案

### CherryUSB

CherryUSB 1.5.3 的 FS + FIFO/PIO 版本曾完成编译，可作回退对照；不作为当前提交的运行栈，避免维护两套 USB API 和描述符。

### emUSB 官方裸机示例

如 RT-Thread 集成继续停在 `ATTACHED|SUSPENDED`，可直接烧录 Infineon 官方 CDC echo 作为板级 USB/VBUS 基准，以区分 BSP 集成问题和物理端口问题。

## 后果

- 可以按“Secure/NS 启动 -> P16.5 心跳 -> KitProg UART -> Linux lsusb -> ttyACM -> READY/PING/PONG”逐层定位。
- 当前文本 PING/PONG 只验证 CDC 链路，不代表后续视觉数据吞吐和实时性已经满足控制要求。
- 当前代码通过主机测试和目标编译，不代表硬件通信已完成；实机结果必须记录供电、接线、固件哈希与安全状态。

## 2026-07-30 台架证据

- 安全状态：电机/执行器不使能，USB 诊断永久输出 `actuator_tx=0`，自动测试不发送 CAN。
- 固件：M33 `text=190096 data=15616 bss=244300`；Secure+NS 写入与 raw/XIP/NS 校验全部通过，且到达 Non-secure reset handler。
- EdgeTalk：蓝灯正常闪；FinSH 为 `state=0x11 configured=0 conn=1 cfg=0`，即 `ATTACHED|SUSPENDED`。
- 树莓派：无 EdgeTalk `lsusb` 条目、无 `/dev/ttyACM*`、内核无设备插拔事件、供电状态正常。
- 物理线材已由用户确认可传输数据；EdgeTalk 官方 `P17.4 VBUS_DETECT` 配置为输入后读低，DWC2 `DCTL=0` 且未收到主机 USB reset。下一步优先核对树莓派 Host 口、EdgeTalk Device 口和板级 VBUS 检测/供电路径。

## 官方来源

- https://github.com/RT-Thread/rt-thread/tree/36f52133ec2e8f7d5c53358bcbd96f2b0e5ac132/components/drivers/usb/cherryusb
- https://github.com/RT-Thread-Studio/sdk-bsp-psoc_e84-edgi-talk/tree/1.1.0/projects/Edgi_Talk_CherryUSB/Edgi_Talk_M33_USB_D
- https://github.com/Infineon/mtb-example-psoc-edge-usb-device-cdc-echo/tree/42fbdaeeac61c8b9eae049855b488f0862d9385c

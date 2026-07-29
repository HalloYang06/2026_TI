# EdgeTalk 与5号电机参考记录

## 1. 参考范围

参考仓库：[`HalloYang06/PSOC_E84_robot`](https://github.com/HalloYang06/PSOC_E84_robot)，核对提交 `4d9fd2a`（`main`，2026-07-29本地浅克隆）。它用于提取 EdgeTalk M33 工具链、CAN bring-up、5号电机身份和烧录验证方法。

该仓库不是本项目的前身。医疗机械臂的 M33/M55/NanoPi 权限边界、关节映射、BLE、ROS、零点、方向、限位和康复控制参数都不迁入 H 题。

## 2. 可复用的硬件事实

参考仓库当前把 `motor_id=5` 标为灵足 RS00，使用 RobStride/灵足私有 29位扩展帧协议。已记录的 bring-up 组合为：

- EdgeTalk M33 的底层外设是 CANFD0，RT-Thread 设备名是 `can0`。
- 电机总线使用 Classic CAN、`1 Mbps`；即使外设支持 CAN FD，也必须关闭 BRS 并发送 Classic CAN 帧。
- 参考代码曾用 direct PDL polling 绕开尚未稳定的 RT-Thread CAN 中断路径。
- Get_ID 是只读探测入口；广播探测 ID `0x7F` 不能误认为真实电机 ID。
- `limit_cur(0x7018)` 是限幅，不是电流命令；电流模式需正确设置 `run_mode(0x7005)` 和 `iq_ref(0x7006)`。
- CSP位置序列涉及 `run_mode`、`limit_spd(0x7017)`、`limit_cur(0x7018)` 和 `loc_ref(0x7016)`；MIT力矩前馈不能直接等同厂家电流模式。

参考工程里5号槽位的 `gear_ratio=1.0`、`direction=+1`、`zero_offset=6.010 rad`、`calibrated=1` 属于原机械臂装配，H题必须重新测量，禁止复制。

## 3. 推荐的 H 题 bring-up 顺序

1. 断开机构负载或保证摆杆机械隔离，确认24 V/额定电压、共地、CANH/CANL和两端约120 Ω终端。
2. 只读取 CAN 控制器状态和错误计数，确认总线 `ERROR-ACTIVE`、无 bus-off、发送不会长期 pending。
3. 发送只读 Get_ID，核对返回的真实 `motor_id` 与 UID；不要先发 enable、zero 或运动目标。
4. 开启只读反馈，验证位置、速度、温度、故障和 fresh/stale 超时语义。
5. 重新标定 H 题机构的电机方向、编码器零点、传动比、角度上下限、死区和回差。
6. 只有在车轮架空、执行器机械状态明确、限流电源、硬件急停和人工断电接管均就绪时，才手工进行极小角度台架测试。

任何自动测试都停在第4步，不使能电机。

## 4. 参考工程的构建入口

M33 工程基于 RT-Thread、SCons、ARM GNU Toolchain 和 Infineon组件。通用入口位于 `firmware/m33/SConstruct`：

```powershell
$env:RTT_EXEC_PATH = '<ARM GNU Toolchain bin目录>'
scons -C firmware/m33 -j4
```

预期主要产物为 `firmware/m33/rt-thread.elf` 和 `firmware/m33/build/rtthread.hex`。`SConstruct` 在 Windows 且工具存在时调用：

```text
firmware/m33/tools/edgeprotecttools/bin/edgeprotecttools.exe
firmware/m33/config/boot_with_extended_boot_scons.json
```

参考仓库的2026-07-03记录使用 RT-Thread Studio 自带 SCons 和 GCC 13.3，成功生成 `build/rtthread.hex`。这些路径是当时电脑的 `F:\RT-ThreadStudio\...`，本项目应改成环境变量或本机配置，不提交机器专用绝对路径。

## 5. 已验证烧录脚本的含义

参考文件 `firmware/m33/tools/flash_m33_verified.ps1` 不是普通“下载按钮”的等价物。它依次：

1. 校验仓库内与 Secure工程的 `s_start_pse84.c` SHA-256一致。
2. 构建 Secure M33 `Debug/rtthread.elf` 并执行 `post-build`。
3. 将 Secure HEX复制到 Edge Protect合并目录。
4. 构建 Non-secure M33和组合 `Debug/rtthread.hex`。
5. 用 `edgeprotecttools hex-relocate` 生成仅供 XIP校验的 `rtthread_xip_verify.hex`。
6. 用 Infineon OpenOCD 2.0.0、KitProg3/SWD和 `PSE84_SMIF.FLM` 写入、raw verify、缓存失效、XIP alias verify，再启动 Non-secure M33。

参考调用形式为：

```powershell
powershell -ExecutionPolicy Bypass -File firmware\m33\tools\flash_m33_verified.ps1
```

脚本硬编码了参考电脑的 RT-Thread Studio、工具链和 OpenOCD路径，且需要相邻 `secureCore` 工程。目标仓库当前没有复制该脚本或第三方工具，不能直接执行这条命令。待 EdgeTalk 工程正式迁入后，应把路径参数化、记录工具版本和校验和，再在人工确认下烧录。

EdgeTalk启动顺序是 Secure M33 -> Non-secure M33 -> 可选 M55。H题第一版不需要 M55；参考烧录脚本也明确设置 `ENABLE_CM55=0` 和 M33 standalone。

## 6. 证据等级

### 官方来源

- [Infineon PSoC Edge E8产品页](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-edge-e8/)
- [RT-Thread SCons构建文档](https://www.rt-thread.io/document/site/programming-manual/scons/scons/)

### 参考仓库工程证据

- `firmware/m33/SConstruct`、`rtconfig.py`：目标、工具链和产物。
- `firmware/m33/tools/flash_m33_verified.ps1` 与 OpenOCD TCL：写入和双路径校验步骤。
- `firmware/m33/docs/FLASH_LOG_20260703.md`：一次构建、写入、verify和串口 smoke记录。
- `firmware/m33/docs/CAN_MOTOR_BRINGUP_RETROSPECTIVE_20260605.md`：Classic CAN 1 Mbps、direct PDL、Get_ID、fresh/stale和物理层诊断复盘。
- `firmware/m33/applications/control/control_layer.*`：RS00私有协议、MIT/CSP/电流模式接口和反馈解析。

参考仓库记录只能证明特定版本、板卡和台架曾工作，不能替代 H题实物的电机铭牌确认、接线核对和重新标定。

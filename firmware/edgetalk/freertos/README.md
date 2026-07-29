# EdgeTalk M55 FreeRTOS shadow运行时

本目录是H题M55正式运行时的板级薄层。它以200 Hz读取M33共享传感器快照、运行LQG控制管线、发布`SHADOW_ONLY`结果，并以10 Hz调用LVGL数据钩子。它不包含CAN驱动、电机协议、执行器发送或机械臂业务。

## 安全边界

- `hball_m55_shadow_task.c`只写M33/M55共享`control_shadow`，固定设置`HBALL_IPC_CONTROL_FLAG_SHADOW_ONLY`。
- `Makefile.hball.mk`不编译`hball_can.c`、M33台架代码或任何执行器适配器。
- 任务创建、堆分配、栈溢出或调度器启动失败时，`main_cm55.c`关中断并进入等待，禁止带病继续。
- 当前M33安全门始终返回`actuator_tx_allowed=false`；自动测试不得添加运动路径。

## 固定依赖

- Infineon FreeRTOS仓库：<https://github.com/Infineon/freertos>
- 标签：`release-v10.6.202`
- 提交：`8a19c8db81becf1e981a5f94630952160fddf8c5`
- 内核：FreeRTOS-Kernel `V10.6.2`
- Cortex-M55 GCC port：`Source/portable/COMPONENT_CM55/TOOLCHAIN_GCC_ARM`
- CMSIS：`release-v6.1.0`
- async-transfer：`release-v1.1.1`

`deps/*.mtb`固定上述依赖。不得用Cortex-M7 port或只改`-mcpu`的临时工程替代官方M55上下文切换代码。

## ModusToolbox集成

在官方PSoC Edge多核工程的`proj_cm55/Makefile`中，于`make/start.mk`之前包含：

```make
include ../../../firmware/edgetalk/freertos/Makefile.hball.mk
```

工程必须提供完整的PSE84 BSP依赖和`mtb-dsl-pse8xxgp` recipe；只导入FreeRTOS会导致`core-make recipe-make not found`。构建变量由片段固定为：

```text
CORE=CM55
CORE_NAME=CM55_0
COMPONENTS+=FREERTOS
VFP_SELECT=hardfp
VFP_SELECT_PRECISION=doublefp
MVE_SELECT=NO_MVE
```

ModusToolbox 3.7的Device Configurator必须同时收到PSE84 DSL和基础device-db。当前PSE846可用的是全局`device-db release-v4.38.0`，旧`4.21.0`不含该器件；离线确认可用的形式为：

```powershell
device-configurator-cli.exe `
  --library "<mtb-dsl-pse8xxgp>/props.json,<global>/device-db/release-v4.38.0/props.json" `
  --build <BSP>/design.modus --readonly
```

FreeRTOS stream buffer会调用`xTaskGetCurrentTaskHandle()`，因此本目录配置固定`INCLUDE_xTaskGetCurrentTaskHandle=1`；ModusToolbox链接变量必须使用`LDLIBS+=-lm`。这两项均由主机契约测试保护。

链接脚本必须放置仓库`linker/hball_dualcore_section.ld.inc`定义的共享段，且M33/M55两侧保持同一地址和大小。

## 验收

主仓库静态和主机编译检查：

```powershell
python -m pytest firmware/edgetalk/tests/test_m55_freertos_contract.py -q
```

真机镜像只有同时满足以下条件才可称为Cortex-M55可部署产物：

```text
compile_commands contains -mcpu=cortex-m55+nomve
ELF Tag_CPU_arch = v8.1-M.mainline
ELF hard-float ABI enabled
.hball_ipc_shared address = 0x261C0000
.hball_ipc_shared size = 0x100
.hball_ipc_shared alignment = 32 bytes
```

2026-07-30的离线单核构建已经满足ARMv8.1-M、hard-float、256字节段和H-ball/FreeRTOS符号检查，但使用官方示例内存设计时该段位于`0x262FC000`；当前M33实测链接图位于`0x261C0000`。因此该ELF只证明真CM55可编译，明确不通过双核地址验收。最终`make`还因缺少匹配的M33 NS HEX停在多核合并阶段，尚无完整可烧录包。

在最终ELF完成上述检查前不得烧录M55。即使镜像通过，也先在电机动力断开、轮子/执行器卸载、仅调试器/USB供电且操作员可直接断电的条件下验证任务频率和IPC计数。

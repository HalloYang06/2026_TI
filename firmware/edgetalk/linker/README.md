# EdgeTalk双核链接约束

`hball_dualcore_section.ld.inc`是M33和M55必须共同采用的GNU ld片段。将其内容放进两个工程`board/linker_scripts/link.ld`的`SECTIONS`内部，并置于任何其他映射到`m33_m55_shared`的输出段之前。

厂商PSoC Edge BSP 1.1.0生成的内存图把双核共享SRAM定义为`0x261C0000`、长度`0x00040000`。片段把H题协议独占的256字节固定在该区域起点，要求32字节对齐并在链接时检查地址和尺寸。M33和M55都链接同一个`.hball_ipc_shared`输入段，因此两边的`hball_ipc_platform_region()`必须解析到同一物理地址。

不同PSE84示例的`design.modus`可能生成不同内存图。2026-07-30的官方CDC多核示例把`m33_m55_shared`定义为`0x262FC000`，而当前已经编译并验证的RT-Thread M33 BSP仍使用`0x261C0000`。二者都是各自设计内的合法地址，但不能混用；禁止只修改C头文件常量或只让一侧链接通过。正式CM55必须复用当前M33的同一份Device Configurator内存设计，并让两侧map文件同时通过下述检查。

不要把对象放进`.cy_sharedmem`：厂商M33和M55链接脚本分别把该段放在`0x240FE000`和`0x240FF000`，两者并不重叠。也不要让两个工程各自从共享区域自由分配，否则新增对象后地址可能漂移。

部署前分别检查：

```powershell
arm-none-eabi-readelf.exe -S rtthread.elf | Select-String 'hball_ipc_shared'
arm-none-eabi-nm.exe -n rtthread.elf | Select-String '__hball_ipc_shared_(start|end)__'
```

M33与M55都必须得到`start=0x261C0000`、`end=0x261C0100`，段类型为`NOBITS`。只有M33在启动时清零区域；M55不得清零，只能在协议头/CRC校验通过后消费数据。

# MSPM0G3507 command-line build and DAPLink flash

This project can now be built with the installed Keil ArmClang toolchain. The
existing TI Clang path (`build-ti.bat`) still requires a separate TI compiler
installation and is not the default path.

## Required tools

- MSPM0 SDK: `build-keil.bat` currently detects
  `F:\TI\mspm0_sdk_2_01_00_03` or `C:\ti\mspm0_sdk_2_11_00_07`.
- Keil ArmClang: the build helper detects the configured
  `KEIL_ARMCLANG_BIN`, the current `D:\Keil5_5_39` installation, or the
  original `F:\keil_v5` installation.
- Python 3.12 plus pyOCD are installed for DAPLink flashing.
- MSPM0G CMSIS Device Pack: save as `F:\TI\packs\TexasInstruments.MSPM0G_DFP.1.1.0.pack`.

SysConfig 1.20.0 is installed at `F:\TI\sysconfig_1.20.0`. It is only needed
after editing `wit-oled-hardware-spi.syscfg`. Until then the checked-in generated
`Debug\ti_msp_dl_config.c/.h` files are reused.

## Commands

Run these in Windows CMD or PowerShell from the project root:

```bat
build-keil.bat
flash-daplink.bat
open-sysconfig.bat
```

The compiled image is `Keil\Objects\wit-oled-hardware-spi.hex`. The generated
`Keil\wit-oled-hardware-spi.uvprojx` can also be opened in uVision.

Wire DAPLink as SWDIO, SWCLK, GND, and VTref/3.3 V; NRST is recommended.
Do not use `pyocd unlock` or a chip-wide mass erase on this MSPM0 target.

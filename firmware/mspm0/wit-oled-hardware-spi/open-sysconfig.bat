@echo off
setlocal

set "SYSCONFIG_GUI=F:\TI\sysconfig_1.20.0\sysconfig_gui.bat"
set "SYSCFG_FILE=%~dp0wit-oled-hardware-spi.syscfg"

if not exist "%SYSCONFIG_GUI%" (
    echo SysConfig was not found: %SYSCONFIG_GUI%
    exit /b 1
)

start "SysConfig" "%SYSCONFIG_GUI%" "%SYSCFG_FILE%"

@echo off
setlocal

set "SDK_ROOT=F:\TI\mspm0_sdk_2_01_00_03"
if not exist "%SDK_ROOT%\.metadata\product.json" set "SDK_ROOT=C:\ti\mspm0_sdk_2_05_01_00"
if not exist "%SDK_ROOT%\.metadata\product.json" set "SDK_ROOT=C:\ti\mspm0_sdk_2_11_00_07"
if not exist "%SDK_ROOT%\.metadata\product.json" (
    echo MSPM0 SDK was not found. Update SDK_ROOT in build-keil.bat.
    exit /b 1
)

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\generate-keil-project.ps1" -ProjectRoot "%~dp0." -SdkRoot "%SDK_ROOT%"
if errorlevel 1 exit /b %errorlevel%

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\build-keil.ps1" -ProjectRoot "%~dp0." -SdkRoot "%SDK_ROOT%"
exit /b %ERRORLEVEL%

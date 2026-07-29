@echo off
setlocal
call "%~dp0tools\ti-env.bat"

if not exist "%~dp0Keil\Objects\wit-oled-hardware-spi.hex" (
    echo Firmware not found. Run build-keil.bat first.
    exit /b 1
)
if not exist "%TI_DFP%" (
    echo MSPM0 CMSIS Device Pack not found: %TI_DFP%
    exit /b 1
)

set "PYTHON=py"
where py >nul 2>nul || set "PYTHON=%LocalAppData%\Programs\Python\Python312\python.exe"
if not exist "%PYTHON%" if not "%PYTHON%"=="py" (
    echo Python was not found. Install Python 3.12 or correct this path.
    exit /b 1
)

%PYTHON% -m pyocd list | findstr /C:"No available debug probes are connected" >nul
if not errorlevel 1 (
    echo No DAPLink/CMSIS-DAP probe is connected.
    exit /b 1
)

%PYTHON% -m pyocd load --pack "%TI_DFP%" -t mspm0g3507 "%~dp0Keil\Objects\wit-oled-hardware-spi.hex"

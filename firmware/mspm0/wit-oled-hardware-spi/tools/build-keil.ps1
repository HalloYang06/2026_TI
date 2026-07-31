param(
    [Parameter(Mandatory = $true)]
    [string]$ProjectRoot,
    [Parameter(Mandatory = $true)]
    [string]$SdkRoot,
    [string]$KeilBin
)

$ErrorActionPreference = 'Stop'

function Invoke-Tool([string]$FilePath, [object[]]$Arguments, [string]$Failure) {
    $startInfo = New-Object System.Diagnostics.ProcessStartInfo
    $startInfo.FileName = $FilePath
    $startInfo.Arguments = ($Arguments -join ' ')
    $startInfo.UseShellExecute = $true
    $process = [System.Diagnostics.Process]::Start($startInfo)
    $process.WaitForExit()
    if ($process.ExitCode -ne 0) {
        throw "$Failure (exit $($process.ExitCode))"
    }
}

$keilCandidates = @(
    $KeilBin,
    $env:KEIL_ARMCLANG_BIN,
    'D:\KEIL\KEILV5\ARM\ARMCLANG\bin',
    'D:\Keil5_5_39\ARM\ARMCLANG\bin',
    'F:\keil_v5\ARM\ARMCLANG\bin'
) | Where-Object { -not [string]::IsNullOrWhiteSpace($_) }

$keilRoot = $keilCandidates |
    Where-Object { Test-Path (Join-Path $_ 'armclang.exe') } |
    Select-Object -First 1
if ([string]::IsNullOrWhiteSpace($keilRoot)) {
    throw "Keil ArmClang was not found. Set KEIL_ARMCLANG_BIN or pass -KeilBin."
}

$compiler = Join-Path $keilRoot 'armclang.exe'
$assembler = Join-Path $keilRoot 'armasm.exe'
$linker = Join-Path $keilRoot 'armlink.exe'
$fromElf = Join-Path $keilRoot 'fromelf.exe'
$outputDir = Join-Path $ProjectRoot 'Keil\Objects'
$listingDir = Join-Path $ProjectRoot 'Keil\Listings'
$repoRoot = [IO.Path]::GetFullPath((Join-Path $ProjectRoot '..\..\..'))
$missionProtocolDir = Join-Path $repoRoot 'shared\protocol'

foreach ($tool in $compiler, $assembler, $linker, $fromElf) {
    if (!(Test-Path $tool)) { throw "Keil tool not found: $tool" }
}
New-Item -ItemType Directory -Force -Path $outputDir, $listingDir | Out-Null

$sources = @(
    'main.c', 'Debug\ti_msp_dl_config.c',
    'App\Control\chassis_actuator.c',
    'Drivers\CAN\hball_can_protocol.c', 'Drivers\CAN\hball_can_recovery.c',
    'Drivers\CAN\hball_can_port.c', 'Drivers\CAN\hball_mission_client.c',
    'App\Mission\hball_mission_menu.c',
    'App\Mission\hball_mission_policy.c',
    'App\Runtime\hball_coop_scheduler.c',
    'App\Runtime\hball_runtime_dispatcher.c',
    'App\Runtime\hball_runtime_services.c',
    'App\Runtime\hball_runtime_target.c',
    (Join-Path $missionProtocolDir 'hball_mission_can.c'),
    'Drivers\ENCODER\encoder.c',
    'Drivers\GRAY\beeper.c', 'Drivers\GRAY\gray.c', 'Drivers\GRAY\key.c',
    'Drivers\GRAY\led.c', 'Drivers\GRAY\track.c', 'Drivers\MOTOR\motor.c',
    'Drivers\MSPM0\clock.c', 'Drivers\MSPM0\interrupt.c',
    'Drivers\OLED_Hardware_SPI\oled_hardware_spi.c', 'Drivers\PID\pid.c',
    'Drivers\UART_VOFA+\uart_vofa.c',
    'Drivers\WIT\wit_jy901s_config.c',
    'Drivers\WIT\wit_parser.c', 'Drivers\WIT\wit_byte_queue.c',
    'Drivers\WIT\wit.c',
    'Drivers\MSPM0\hball_stack_reserve.s',
    "$SdkRoot\source\ti\devices\msp\m0p\startup_system_files\keil\startup_mspm0g350x_uvision.s"
)
$includeDirs = @(
    "$SdkRoot\source", "$SdkRoot\source\third_party\CMSIS\Core\Include",
    $ProjectRoot, (Join-Path $ProjectRoot 'Debug'),
    (Join-Path $ProjectRoot 'App\Control'),
    (Join-Path $ProjectRoot 'App\Mission'),
    (Join-Path $ProjectRoot 'App\Runtime'), $missionProtocolDir
) + (Get-ChildItem (Join-Path $ProjectRoot 'Drivers') -Directory | ForEach-Object FullName)
$compileArgs = @('--target=arm-arm-none-eabi', '-mcpu=cortex-m0plus', '-mthumb', '-O0', '-fshort-wchar', '-fshort-enums', '-D__MSPM0G3507__') +
    ($includeDirs | ForEach-Object { "-I$_" })

$objects = @()
foreach ($source in $sources) {
    $sourcePath = if ([IO.Path]::IsPathRooted($source)) { $source } else { Join-Path $ProjectRoot $source }
    $objectName = ([IO.Path]::GetFileNameWithoutExtension($sourcePath) + '-' + ([Math]::Abs($sourcePath.ToLowerInvariant().GetHashCode())) + '.o')
    $objectPath = Join-Path $outputDir $objectName
    if ($sourcePath.EndsWith('.s', [StringComparison]::OrdinalIgnoreCase)) {
        Invoke-Tool $assembler @('--cpu', 'Cortex-M0+', $sourcePath, '-o', $objectPath) "Assembly failed: $source"
    } else {
        Invoke-Tool $compiler ($compileArgs + @('-c', $sourcePath, '-o', $objectPath)) "Compilation failed: $source"
    }
    $objects += $objectPath
}

$elf = Join-Path $outputDir 'wit-oled-hardware-spi.axf'
$map = Join-Path $listingDir 'wit-oled-hardware-spi.map'
$driverLib = Join-Path $SdkRoot 'source\ti\driverlib\lib\keil\m0p\mspm0g1x0x_g3x0x\driverlib.a'
Invoke-Tool $linker (@('--cpu', 'Cortex-M0+', '--strict', '--keep=hball_stack_extension', "--scatter=$(Join-Path $ProjectRoot 'Keil\mspm0g3507.sct')", '--summary_stderr', '--info', 'summarysizes', '--map', "--list=$map", "--output=$elf") + $objects + @($driverLib)) 'Link failed.'

$minimumStackBytes = 0x800
$symbolTable = & $fromElf --text -s $elf
if ($LASTEXITCODE -ne 0) {
    throw "Unable to inspect linked stack layout (exit $LASTEXITCODE)"
}
$symbols = $symbolTable -join "`n"
$stackMatches = [regex]::Matches(
    $symbols,
    '(?m)^\s+\d+\s+STACK\s+0x([0-9a-fA-F]+)\s+.*\s+0x([0-9a-fA-F]+)\s*$'
)
$initialSpMatch = [regex]::Match(
    $symbols,
    '(?m)^\s+\d+\s+__initial_sp\s+0x([0-9a-fA-F]+)\s+'
)
if (($stackMatches.Count -eq 0) -or !$initialSpMatch.Success) {
    throw 'Linked STACK or __initial_sp symbol was not found.'
}
$stackSegments = @($stackMatches | ForEach-Object {
    [PSCustomObject]@{
        Base = [Convert]::ToUInt32($_.Groups[1].Value, 16)
        Size = [Convert]::ToUInt32($_.Groups[2].Value, 16)
    }
} | Sort-Object Base)
$stackBase = $stackSegments[0].Base
$stackEnd = $stackBase
$stackBytes = 0
foreach ($segment in $stackSegments) {
    if ($segment.Base -ne $stackEnd) {
        throw 'Non-contiguous target STACK sections.'
    }
    $stackBytes += $segment.Size
    $stackEnd += $segment.Size
}
$initialSp = [Convert]::ToUInt32($initialSpMatch.Groups[1].Value, 16)
if (($stackBytes -lt $minimumStackBytes) -or
    ($initialSp -ne $stackEnd)) {
    throw ("Unsafe target stack layout: base=0x{0:X8}, size=0x{1:X}, " +
           "initial_sp=0x{2:X8}") -f $stackBase, $stackBytes, $initialSp
}
Write-Host ("Verified target stack: 0x{0:X} bytes" -f $stackBytes)

Invoke-Tool $fromElf @('--i32', '--output', (Join-Path $outputDir 'wit-oled-hardware-spi.hex'), $elf) 'HEX conversion failed.'
Write-Host "Built $elf"

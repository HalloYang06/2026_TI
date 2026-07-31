param(
    [Parameter(Mandatory = $true)]
    [string]$ProjectRoot,
    [Parameter(Mandatory = $true)]
    [string]$SdkRoot
)

$ErrorActionPreference = 'Stop'
$keilDir = Join-Path $ProjectRoot 'Keil'
$template = Join-Path $SdkRoot 'project\keil\project.uvprojx'
$scatterTemplate = Join-Path $SdkRoot 'project\keil\mspm0g3507.sct'
$projectFile = Join-Path $keilDir 'wit-oled-hardware-spi.uvprojx'
$scatterFile = Join-Path $keilDir 'mspm0g3507.sct'

New-Item -ItemType Directory -Path $keilDir -Force | Out-Null
if (Test-Path $template) {
    Copy-Item $template $projectFile -Force
} elseif (!(Test-Path $projectFile)) {
    throw "Neither SDK template nor existing Keil project was found."
} else {
    Write-Host "SDK template is unavailable; reusing existing $projectFile"
}

if (Test-Path $scatterTemplate) {
    Copy-Item $scatterTemplate $scatterFile -Force
} elseif (!(Test-Path $scatterFile)) {
    throw "Neither SDK scatter template nor existing scatter file was found."
} else {
    Write-Host "SDK scatter template is unavailable; reusing existing $scatterFile"
}

[xml]$project = Get-Content $projectFile
$target = $project.Project.Targets.Target
$target.TargetName = 'MSPM0G3507'
$target.TargetOption.TargetCommonOption.OutputDirectory = '.\Objects\'
$target.TargetOption.TargetCommonOption.OutputName = 'wit-oled-hardware-spi'
$target.TargetOption.TargetCommonOption.CreateHexFile = '1'
$target.TargetOption.TargetArmAds.Cads.VariousControls.Define = '__MSPM0G3507__'
$include = @(
    "$SdkRoot\source",
    "$SdkRoot\source\third_party\CMSIS\Core\Include",
    '..', '..\Debug', '..\App\Mission', '..\App\Runtime',
    '..\Drivers\CAN', '..\Drivers\ENCODER', '..\Drivers\GRAY', '..\Drivers\MOTOR',
    '..\Drivers\MSPM0', '..\Drivers\OLED_Hardware_SPI', '..\Drivers\PID',
    '..\Drivers\UART_VOFA+', '..\Drivers\WIT',
    '..\..\..\..\shared\protocol'
) -join ';'
$target.TargetOption.TargetArmAds.Cads.VariousControls.IncludePath = $include
$target.TargetOption.TargetArmAds.LDads.ScatterFile = '.\mspm0g3507.sct'

$groups = $target.Groups
$groups.RemoveAll()

function Add-Group([string]$name, [object]$files) {
    $group = $project.CreateElement('Group')
    $groupName = $project.CreateElement('GroupName')
    $groupName.InnerText = $name
    [void]$group.AppendChild($groupName)
    $fileList = $project.CreateElement('Files')
    foreach ($entry in $files) {
        $file = $project.CreateElement('File')
        foreach ($part in @(@('FileName', $entry[0]), @('FileType', $entry[1]), @('FilePath', $entry[2]))) {
            $node = $project.CreateElement($part[0])
            $node.InnerText = $part[1]
            [void]$file.AppendChild($node)
        }
        [void]$fileList.AppendChild($file)
    }
    [void]$group.AppendChild($fileList)
    [void]$groups.AppendChild($group)
}

$libraryFiles = @(
    ,@('driverlib.a', '2', "$SdkRoot\source\ti\driverlib\lib\keil\m0p\mspm0g1x0x_g3x0x\driverlib.a")
)
$applicationFiles = @(
    ,@('startup_mspm0g350x_uvision.s', '2', "$SdkRoot\source\ti\devices\msp\m0p\startup_system_files\keil\startup_mspm0g350x_uvision.s")
    ,@('main.c', '1', '..\main.c')
    ,@('ti_msp_dl_config.c', '1', '..\Debug\ti_msp_dl_config.c')
    ,@('hball_can_protocol.c', '1', '..\Drivers\CAN\hball_can_protocol.c')
    ,@('hball_can_recovery.c', '1', '..\Drivers\CAN\hball_can_recovery.c')
    ,@('hball_can_port.c', '1', '..\Drivers\CAN\hball_can_port.c')
    ,@('hball_mission_client.c', '1', '..\Drivers\CAN\hball_mission_client.c')
    ,@('hball_mission_menu.c', '1', '..\App\Mission\hball_mission_menu.c')
    ,@('hball_mission_policy.c', '1', '..\App\Mission\hball_mission_policy.c')
    ,@('hball_runtime_services.c', '1', '..\App\Runtime\hball_runtime_services.c')
    ,@('hball_mission_can.c', '1', '..\..\..\..\shared\protocol\hball_mission_can.c')
    ,@('encoder.c', '1', '..\Drivers\ENCODER\encoder.c')
    ,@('beeper.c', '1', '..\Drivers\GRAY\beeper.c')
    ,@('gray.c', '1', '..\Drivers\GRAY\gray.c')
    ,@('key.c', '1', '..\Drivers\GRAY\key.c')
    ,@('led.c', '1', '..\Drivers\GRAY\led.c')
    ,@('track.c', '1', '..\Drivers\GRAY\track.c')
    ,@('motor.c', '1', '..\Drivers\MOTOR\motor.c')
    ,@('clock.c', '1', '..\Drivers\MSPM0\clock.c')
    ,@('interrupt.c', '1', '..\Drivers\MSPM0\interrupt.c')
    ,@('oled_hardware_spi.c', '1', '..\Drivers\OLED_Hardware_SPI\oled_hardware_spi.c')
    ,@('pid.c', '1', '..\Drivers\PID\pid.c')
    ,@('uart_vofa.c', '1', '..\Drivers\UART_VOFA+\uart_vofa.c')
    ,@('wit_jy901s_config.c', '1', '..\Drivers\WIT\wit_jy901s_config.c')
    ,@('wit_parser.c', '1', '..\Drivers\WIT\wit_parser.c')
    ,@('wit.c', '1', '..\Drivers\WIT\wit.c')
    ,@('wit-oled-hardware-spi.syscfg', '5', '..\wit-oled-hardware-spi.syscfg')
)
Add-Group -name 'library' -files $libraryFiles
Add-Group -name 'application' -files $applicationFiles

$writerSettings = New-Object System.Xml.XmlWriterSettings
$writerSettings.Encoding = New-Object System.Text.UTF8Encoding($false)
$writerSettings.Indent = $true
$writer = [System.Xml.XmlWriter]::Create($projectFile, $writerSettings)
$project.Save($writer)
$writer.Dispose()
Write-Host "Generated $projectFile"

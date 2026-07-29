param(
    [string]$PortName = "COM27",
    [int]$BaudRate = 9600,
    [int]$DurationSeconds = 60
)

$serialPort = [System.IO.Ports.SerialPort]::new(
    $PortName,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)

try {
    $serialPort.Open()
    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt $DurationSeconds) {
        $chunk = $serialPort.ReadExisting()
        if (-not [string]::IsNullOrEmpty($chunk)) {
            [Console]::Out.Write($chunk)
            [Console]::Out.Flush()
        }
        Start-Sleep -Milliseconds 25
    }
}
finally {
    if ($serialPort.IsOpen) {
        $serialPort.Close()
    }
    $serialPort.Dispose()
}

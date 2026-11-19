[CmdletBinding()]
param(
    [Parameter(Mandatory)]
    [ValidatePattern('^COM[1-9][0-9]*$')]
    [string]$Port,

    [ValidateRange(1, 3600)]
    [int]$Seconds = 30,

    [string]$OutputPath
)

$ErrorActionPreference = 'Stop'
if (!$OutputPath) {
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
    $OutputPath = Join-Path $PSScriptRoot "../mcu/build/tuning-$stamp.log"
}
$OutputPath = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($OutputPath)
[void][IO.Directory]::CreateDirectory([IO.Path]::GetDirectoryName($OutputPath))

$serial = [IO.Ports.SerialPort]::new($Port, 115200, [IO.Ports.Parity]::None, 8, [IO.Ports.StopBits]::One)
$serial.Handshake = [IO.Ports.Handshake]::None
$serial.DtrEnable = $false
$serial.RtsEnable = $false
$serial.ReadTimeout = 50

$capture = $null
$buffer = [byte[]]::new(1024)
$bytesWritten = 0L
try {
    $serial.Open()
    $capture = [IO.File]::Open($OutputPath, [IO.FileMode]::CreateNew, [IO.FileAccess]::Write, [IO.FileShare]::Read)
    Write-Host "Capturing $Port at 115200 8N1 for $Seconds s -> $OutputPath"
    $watch = [Diagnostics.Stopwatch]::StartNew()
    while ($watch.Elapsed.TotalSeconds -lt $Seconds) {
        $available = [Math]::Min($serial.BytesToRead, $buffer.Length)
        if ($available -eq 0) {
            Start-Sleep -Milliseconds 10
            continue
        }
        try {
            $count = $serial.Read($buffer, 0, $available)
        }
        catch [TimeoutException] {
            continue
        }
        # Preserve text logs, telemetry, and partial lines exactly as received.
        $capture.Write($buffer, 0, $count)
        $capture.Flush()
        $bytesWritten += $count
    }
}
finally {
    if ($null -ne $capture) { $capture.Dispose() }
    $serial.Dispose()
}

Write-Host "Captured $bytesWritten bytes -> $OutputPath"
if ($bytesWritten -gt 0) {
    Get-Content -LiteralPath $OutputPath -Encoding UTF8 -Tail 8
}

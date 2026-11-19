[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,
    [ValidateRange(1, 1000)]
    [int]$Blocks = 90,
    [ValidateRange(1, 1000)]
    [int]$Count = 100,
    [ValidateRange(5, 60000)]
    [int]$IntervalMs = 100
)

$ErrorActionPreference = 'Stop'
$workspace = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$reader = Join-Path $workspace '.agents/skills/mcu-variables/scripts/read.js'
$captureDirectory = (New-Item -ItemType Directory -Force -Path $OutputDirectory).FullName
$stopFile = Join-Path $captureDirectory 'STOP'
$capturePrefix = Get-Date -Format 'yyyyMMdd-HHmmss-fff'

$sampleVariables = @(
    'app_fsm.c::s_app_fsm.current_state',
    'app_fsm.c::s_app_fsm.fault_latched',
    'chassis.c::s_output_enabled',
    'remote.c::s_remote_state.online',
    'remote.c::s_remote_state.rc_data.channel[*]',
    'odom.c::s_odom.odom_interface.x',
    'odom.c::s_odom.odom_interface.y',
    'odom.c::s_odom.angle.z',
    'odom.c::s_odom.wheel_raw.x',
    'odom.c::s_odom.wheel_raw.y',
    'odom.c::s_odom.wheel_raw.z',
    'odom.c::s_odom.wheel_corrected.x',
    'odom.c::s_odom.wheel_corrected.y',
    'odom.c::s_odom.wheel_corrected.z',
    'chassis.c::s_chassis.kine.control.vx',
    'chassis.c::s_chassis.kine.control.vy',
    'chassis.c::s_chassis.kine.control.wz',
    'app_odom_test.c::s_odom_test.segment_index',
    'app_odom_test.c::s_odom_test.phase',
    'app_odom_test.c::s_odom_test.remaining_m',
    'odom.c::s_odom.wheel_valid',
    'odom.c::s_odom.wheel_sample_sequence',
    'odom.c::s_odom.sample_time_us'
)

Write-Host "Capture directory: $captureDirectory"
Write-Host "Blocks=$Blocks Count=$Count IntervalMs=$IntervalMs"
Write-Host "Create STOP in the capture directory to stop at the next block boundary."

if (Test-Path -LiteralPath $stopFile) {
    Write-Host "STOP found; no capture started: $stopFile"
    return
}

for ($block = 1; $block -le $Blocks; $block++) {
    if (Test-Path -LiteralPath $stopFile) {
        Write-Host "STOP found; capture ended at block boundary: $stopFile"
        return
    }

    $captureFile = Join-Path $captureDirectory "$capturePrefix-block-$('{0:D4}' -f $block).ndjson"
    $stderrFile = Join-Path $captureDirectory "$capturePrefix-block-$('{0:D4}' -f $block).stderr.log"
    if ((Test-Path -LiteralPath $captureFile) -or (Test-Path -LiteralPath $stderrFile)) {
        throw "Capture already exists; refusing to overwrite: $captureFile"
    }

    $readerArguments = @($reader, '--workspace', $workspace,
        '--variables', ($sampleVariables -join ','),
        '--count', "$Count", '--interval', "$IntervalMs")
    & node @readerArguments 2> $stderrFile |
        Set-Content -LiteralPath $captureFile -Encoding utf8
    if ($LASTEXITCODE -ne 0) {
        throw "read.js exited with code $LASTEXITCODE; retained $captureFile and $stderrFile"
    }
    if (Select-String -LiteralPath $captureFile -Pattern '"type":"variableErrors"' -Quiet) {
        throw "Variable errors were recorded; inspect $captureFile before using the data"
    }

    Write-Host "$(Get-Date -Format 'HH:mm:ss') block $block/$Blocks saved -> $captureFile"
    if ($block -lt $Blocks) {
        Start-Sleep -Milliseconds 500
    }
}

Write-Host "Completed $Blocks blocks; captures saved in $captureDirectory"

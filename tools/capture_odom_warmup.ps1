[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$OutputDirectory,
    [ValidateRange(1, 2147483647)]
    [int]$Blocks = 60,
    [ValidateRange(1, 1000)]
    [int]$Count = 150,
    [ValidateRange(5, 60000)]
    [int]$IntervalMs = 200
)

$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$mcuWorkspace = Join-Path $repoRoot 'mcu'
$reader = Join-Path $mcuWorkspace '.agents/skills/mcu-variables/scripts/read.js'
$captureDirectory = (New-Item -ItemType Directory -Force -Path $OutputDirectory).FullName
$stopFile = Join-Path $captureDirectory 'STOP'
$capturePrefix = Get-Date -Format 'yyyyMMdd-HHmmss-fff'

$snapshotVariables = @(
    'odom.c::s_odom',
    'bmi088.c::s_bmi088_attitude',
    'bmi088.c::s_bmi088_sample',
    'chassis.c::s_chassis.kine.control',
    'chassis.c::s_output_enabled',
    'app_fsm.c::s_app_fsm.current_state',
    'remote.c::s_remote_state'
)

# Odom publishes raw gyro, acceleration and temperature before the calibration gate.
# Each trend request is a scalar leaf; whole composites are read only in the snapshot.
$sampleVariables = @(
    'odom.c::s_odom.gyro.x',
    'odom.c::s_odom.gyro.y',
    'odom.c::s_odom.gyro.z',
    'odom.c::s_odom.gyro_corrected.x',
    'odom.c::s_odom.gyro_corrected.y',
    'odom.c::s_odom.gyro_corrected.z',
    'odom.c::s_odom.gyro_bias_effective.x',
    'odom.c::s_odom.gyro_bias_effective.y',
    'odom.c::s_odom.gyro_bias_effective.z',
    'odom.c::s_odom.temperature_c',
    'odom.c::s_odom.temperature_valid',
    'odom.c::s_odom.sample_time_us',
    'odom.c::s_odom.acc.x',
    'odom.c::s_odom.acc.y',
    'odom.c::s_odom.acc.z',
    'odom.c::s_odom.wheel_corrected.x',
    'odom.c::s_odom.wheel_corrected.y',
    'odom.c::s_odom.wheel_corrected.z',
    'odom.c::s_odom.wheel_valid',
    'odom.c::s_odom.imu_calibrated',
    'odom.c::s_fusion.stationary',
    'stm32_hal_bmi088.c::s_bmi088_spi_failure_count',
    'odom.c::s_odom.discontinuity_count',
    'chassis.c::s_chassis.kine.control.vx',
    'chassis.c::s_chassis.kine.control.vy',
    'chassis.c::s_chassis.kine.control.wz',
    'chassis.c::s_output_enabled',
    'bmi088.c::s_bmi088_attitude.gyro_bias_residual.x',
    'bmi088.c::s_bmi088_attitude.gyro_bias_residual.y',
    'bmi088.c::s_bmi088_attitude.gyro_bias_residual.z',
    'odom.c::s_odom.angle.z',
    'odom.c::s_odom.imu_ready'
)

function Save-ReadResult {
    param([string]$Name, [string[]]$Variables, [switch]$Sample)

    $captureFile = Join-Path $captureDirectory "$capturePrefix-$Name.ndjson"
    $stderrFile = Join-Path $captureDirectory "$capturePrefix-$Name.stderr.log"
    if ((Test-Path -LiteralPath $captureFile) -or (Test-Path -LiteralPath $stderrFile)) {
        throw "Capture already exists; refusing to overwrite: $captureFile"
    }
    $readerArguments = @($reader, '--workspace', $mcuWorkspace,
        '--variables', ($Variables -join ','))
    if ($Sample) {
        $readerArguments += @('--count', "$Count", '--interval', "$IntervalMs")
    }

    & node @readerArguments 2> $stderrFile |
        Set-Content -LiteralPath $captureFile -Encoding utf8
    if ($LASTEXITCODE -ne 0) {
        throw "read.js exited with code $LASTEXITCODE; retained $captureFile and $stderrFile"
    }

    Write-Host "$(Get-Date -Format 'HH:mm:ss') $Name completed -> $captureFile"
}

if (Test-Path -LiteralPath $stopFile) {
    Write-Host "STOP found; no capture started: $stopFile"
    return
}
Save-ReadResult -Name 'start' -Variables $snapshotVariables
for ($block = 1; $block -le $Blocks; $block++) {
    if (Test-Path -LiteralPath $stopFile) {
        Write-Host "STOP found at block boundary; capture ended: $stopFile"
        return
    }
    Save-ReadResult -Name ('block-{0:D4}' -f $block) -Variables $sampleVariables -Sample
}
Write-Host "Completed $Blocks blocks; captures saved in $captureDirectory"

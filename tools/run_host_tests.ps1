param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$testOutput = Join-Path $repoRoot 'mcu/build/HostTests'
New-Item -ItemType Directory -Force -Path $testOutput | Out-Null
$sdk = 'external/Embedded-Electronic-Control-Standard/sdks'
$includes = @('mcu/src/app', 'mcu/src/service', 'mcu/src/domain', 'mcu/src/domain/kalman',
    "$sdk/infra", "$sdk/domain", "$sdk/device", "$sdk/device/imu", "$sdk/device/bus_motor",
    "$sdk/device/fs_ia10b")
$flags = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror')
$flags += $includes | ForEach-Object { '-I' + (Join-Path $repoRoot $_) }
$fusion = @('mcu/src/domain/kalman/chassis_imu_odom.c',
    'mcu/src/domain/kalman/kalman.c', "$sdk/infra/matrix.c")
$cases = [ordered]@{
    test_imu = @("$sdk/device/imu/imu_attitude.c", "$sdk/device/imu/bmi088.c", "$sdk/device/imu/imu.c")
    test_wheel_feedback = @('mcu/src/service/chassis.c', "$sdk/domain/steer_wheel_kine.c", "$sdk/infra/log.c")
    test_odom_fusion = $fusion
    test_odom = @('mcu/src/service/odom.c', "$sdk/device/imu/imu.c") + $fusion
    test_app_runtime = @('mcu/src/app/app_runtime.c', 'mcu/src/app/app_fsm.c',
        "$sdk/infra/hfsm/hfsm.c", "$sdk/infra/hfsm/hfsm_core.c", "$sdk/infra/log.c")
}
foreach($testName in $cases.Keys) {
    $executable = Join-Path $testOutput ($testName + '.exe')
    $sources = @("tools/tests/$testName.c") + $cases[$testName]
    $arguments = $flags + @($sources | ForEach-Object { Join-Path $repoRoot $_ })
    & $Compiler @arguments '-lm' '-o' $executable
    if($LASTEXITCODE -ne 0) { throw "$testName compilation failed." }
    & $executable
    if($LASTEXITCODE -ne 0) { throw "$testName failed." }
}

param([string]$Compiler = '', [string]$OutputDirectory = 'build/OpticalFlow')
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$mcuRoot = Join-Path $projectRoot 'mcu'
if(!$Compiler) {
    $eideCompiler = Join-Path $env:USERPROFILE '.eide/tools/gcc_arm/bin/arm-none-eabi-gcc.exe'
    if(Test-Path -LiteralPath $eideCompiler) { $Compiler = $eideCompiler }
    else { $Compiler = (Get-Command arm-none-eabi-gcc -ErrorAction Stop).Source }
}
$compilerBin = Split-Path -Parent $Compiler
$objcopy = Join-Path $compilerBin 'arm-none-eabi-objcopy.exe'
$size = Join-Path $compilerBin 'arm-none-eabi-size.exe'
$output = [IO.Path]::GetFullPath((Join-Path $mcuRoot $OutputDirectory))
if(!$output.StartsWith($mcuRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'OutputDirectory must be inside mcu.'
}
New-Item -ItemType Directory -Force -Path $output | Out-Null
$includes = @(
    'Core/Inc', 'Drivers/STM32H7xx_HAL_Driver/Inc', 'Drivers/STM32H7xx_HAL_Driver/Inc/Legacy',
    'Drivers/CMSIS/Device/ST/STM32H7xx/Include', 'Drivers/CMSIS/Include',
    'src/app', 'src/domain', 'src/platform', 'src/service',
    '../external/Embedded-Electronic-Control-Standard/sdks/device',
    '../external/Embedded-Electronic-Control-Standard/sdks/device/bus_motor',
    '../external/Embedded-Electronic-Control-Standard/sdks/device/fs_ia10b',
    '../external/Embedded-Electronic-Control-Standard/sdks/device/imu',
    '../external/Embedded-Electronic-Control-Standard/sdks/device/rgb_led',
    '../external/Embedded-Electronic-Control-Standard/sdks/domain',
    '../external/Embedded-Electronic-Control-Standard/sdks/infra'
)
$flags = @('-mcpu=cortex-m7', '-mfpu=fpv5-sp-d16', '-mfloat-abi=hard', '-mthumb',
    '-g3', '-Og', '-std=gnu11', '-ffunction-sections', '-fdata-sections', '-Wall',
    '-DDEBUG', '-DUSE_PWR_LDO_SUPPLY', '-DUSE_HAL_DRIVER', '-DSTM32H723xx')
$flags += $includes | ForEach-Object { '-I' + $_ }
$sources = @()
foreach($directory in @('Core/Src', 'src', 'Drivers/STM32H7xx_HAL_Driver/Src')) {
    $sources += Get-ChildItem -LiteralPath (Join-Path $mcuRoot $directory) -Filter '*.c' -Recurse |
        Where-Object { $_.Name -notmatch '_template\.c$' } | ForEach-Object { $_.FullName }
}
$sources += Get-ChildItem -LiteralPath (Join-Path $mcuRoot 'Core/Startup') -Filter '*.s' |
    ForEach-Object { $_.FullName }
# Reuse the project's selected Standard SDK files; do not compile unrelated SDK examples.
foreach($line in Get-Content -LiteralPath (Join-Path $mcuRoot '.eide/eide.yml')) {
    if($line -match '^\s*- path: (.+\.c)$') { $sources += [IO.Path]::GetFullPath((Join-Path $mcuRoot $Matches[1])) }
}
$sources = @($sources | Sort-Object -Unique)
$objects = @()
Push-Location $mcuRoot
try {
    $index = 0
    foreach($source in $sources) {
        $object = Join-Path $output ('{0:D3}_{1}.o' -f $index, [IO.Path]::GetFileNameWithoutExtension($source))
        & $Compiler @flags '-c' $source '-o' $object
        if($LASTEXITCODE -ne 0) { throw "Compile failed: $source" }
        $objects += $object
        $index++
        if($index % 25 -eq 0) { Write-Host "Compiled $index / $($sources.Count)" }
    }
    $elf = Join-Path $output 'robot.elf'
    $linkArgs = @('-mcpu=cortex-m7', '-mfpu=fpv5-sp-d16', '-mfloat-abi=hard', '-mthumb',
        '-T', 'STM32H723VGTX_FLASH_USER.ld', '-Wl,--gc-sections', '-Wl,--print-memory-usage',
        '-u', '_printf_float', '--specs=nano.specs', '--specs=nosys.specs',
        ('-Wl,-Map=' + (Join-Path $output 'robot.map')), '-o', $elf)
    $linkArgs += $objects
    $linkArgs += @('-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group')
    $response = Join-Path $output 'link.rsp'
    ($linkArgs | ForEach-Object { '"' + $_.Replace('\', '/') + '"' }) |
        Set-Content -LiteralPath $response -Encoding ascii
    & $Compiler ('@' + $response)
    if($LASTEXITCODE -ne 0) { throw 'Link failed.' }
    & $objcopy '-O' 'ihex' $elf (Join-Path $output 'robot.hex')
    if($LASTEXITCODE -ne 0) { throw 'HEX generation failed.' }
    & $objcopy '-O' 'binary' $elf (Join-Path $output 'robot.bin')
    if($LASTEXITCODE -ne 0) { throw 'BIN generation failed.' }
    & $size $elf
    Write-Host "Firmware ready: $output"
}
finally { Pop-Location }

param(
  [string]$ToolchainBin = '',
  [switch]$AutoSpeed
)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
if (!$ToolchainBin) {
  $compiler = Get-ChildItem 'C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins\*gnu-tools*\tools\bin\arm-none-eabi-gcc.exe' | Select-Object -First 1
  if (!$compiler) { throw 'Specify -ToolchainBin with the ARM GNU toolchain bin directory.' }
  $ToolchainBin = $compiler.DirectoryName
}
$gcc = Join-Path $ToolchainBin 'arm-none-eabi-gcc.exe'
$variant = if ($AutoSpeed) { 'auto' } else { 'manual' }
$outDir = Join-Path $projectRoot "build\$variant"
New-Item -ItemType Directory -Force -Path $outDir | Out-Null
$includes = @('Core/Inc','Core/Grbl','USB_DEVICE/App','USB_DEVICE/Target',
 'Drivers/STM32F1xx_HAL_Driver/Inc','Drivers/STM32F1xx_HAL_Driver/Inc/Legacy',
 'Drivers/CMSIS/Device/ST/STM32F1xx/Include','Drivers/CMSIS/Include',
 'Middlewares/ST/STM32_USB_Device_Library/Core/Inc',
 'Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc')
$flags = @('-mcpu=cortex-m3','-mthumb','-mfloat-abi=soft','-std=gnu11','-Os','-g3',
 '-ffunction-sections','-fdata-sections','-Wall','-Wextra','-DUSE_HAL_DRIVER','-DSTM32F103xE')
if ($AutoSpeed) { $flags += '-DEMBROIDERY_AUTO_SPEED=1' }
foreach ($inc in $includes) { $flags += '-I' + (Join-Path $projectRoot $inc) }
$objects = @()
foreach ($dir in @('Core','Drivers','Middlewares','USB_DEVICE')) {
  foreach ($source in Get-ChildItem (Join-Path $projectRoot $dir) -Recurse -File | Where-Object { $_.Extension -in '.c','.s' }) {
    $relative = $source.FullName.Substring($projectRoot.Length + 1)
    $object = Join-Path $outDir (($relative -replace '[\\/]','_') + '.o')
    & $gcc @flags -c $source.FullName -o $object
    if ($LASTEXITCODE) { throw "Compilation failed: $relative" }
    $objects += $object
  }
}
$elf = Join-Path $outDir 'Vyshivatel.elf'
& $gcc @objects -mcpu=cortex-m3 -mthumb -mfloat-abi=soft --specs=nano.specs --specs=nosys.specs "-T$(Join-Path $projectRoot 'STM32F103RCTX_FLASH.ld')" '-Wl,--gc-sections' "-Wl,-Map=$(Join-Path $outDir 'Vyshivatel.map')" '-Wl,--start-group' -lc -lm '-Wl,--end-group' -o $elf
if ($LASTEXITCODE) { throw 'Link failed' }
& (Join-Path $ToolchainBin 'arm-none-eabi-objcopy.exe') -O ihex $elf (Join-Path $outDir 'Vyshivatel.hex')
if ($LASTEXITCODE) { throw 'HEX generation failed' }
& (Join-Path $ToolchainBin 'arm-none-eabi-size.exe') $elf
if ($LASTEXITCODE) { throw 'Size check failed' }
Write-Output "Built $outDir (no flashing performed)"

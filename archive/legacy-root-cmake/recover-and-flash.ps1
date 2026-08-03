param(
  [Parameter(Mandatory = $false)]
  [string]$ElfPath = (Join-Path $PSScriptRoot 'cmake-build-cmake4stm32\stm32workspace.elf'),

  [Parameter(Mandatory = $false)]
  [ValidateRange(10, 600)]
  [int]$TimeoutSeconds = 180
)

$ErrorActionPreference = 'Stop'

$openOcdRoot = 'D:\openocd-20260302\OpenOCD-20260302-0.12.0'
$openOcdExe = Join-Path $openOcdRoot 'bin\openocd.exe'
$openOcdScripts = Join-Path $openOcdRoot 'share\openocd\scripts'
$openOcdConfig = Join-Path $PSScriptRoot 'stlink.cfg'

foreach ($requiredPath in @($openOcdExe, $openOcdScripts, $openOcdConfig, $ElfPath)) {
  if (-not (Test-Path -LiteralPath $requiredPath)) {
    throw "Required path not found: $requiredPath"
  }
}

$elfFullPath = (Resolve-Path -LiteralPath $ElfPath).Path.Replace('\', '/')
$programCommand = "program {$elfFullPath} verify exit"
$deadline = (Get-Date).AddSeconds($TimeoutSeconds)
$attempt = 0

Write-Host 'Manual RESET recovery window is active.'
Write-Host 'Press and release the STM32 board RESET button until programming starts.'
Write-Host 'OpenOCD retries automatically as soon as SWD becomes available.'

while ((Get-Date) -lt $deadline) {
  $attempt++
  Write-Host "Recovery attempt $attempt..."

  & $openOcdExe `
    -s $openOcdScripts `
    -f $openOcdConfig `
    -c $programCommand

  if ($LASTEXITCODE -eq 0) {
    Write-Host 'Flash and verification completed successfully.'
    Write-Host 'Press and release RESET once more to run the new firmware.'
    [Console]::Beep(1200, 300)
    exit 0
  }

  Start-Sleep -Milliseconds 200
}

throw "Recovery timed out after $TimeoutSeconds seconds without an SWD connection."

param(
  [Parameter(Mandatory = $false)]
  [string]$ElfPath = (Join-Path $PSScriptRoot 'build\Debug\stm32workspace.elf'),

  [Parameter(Mandatory = $false)]
  [ValidateRange(10, 600)]
  [int]$TimeoutSeconds = 180,

  [Parameter(Mandatory = $false)]
  [string]$OpenOcdRoot = $env:OPENOCD_ROOT
)

$ErrorActionPreference = 'Stop'

if (-not $OpenOcdRoot) {
  $openOcdCommand = Get-Command openocd.exe -ErrorAction SilentlyContinue
  if ($openOcdCommand) {
    $OpenOcdRoot = Split-Path -Parent (Split-Path -Parent $openOcdCommand.Source)
  }
}
if (-not $OpenOcdRoot) {
  throw 'OpenOCD root not provided. Pass -OpenOcdRoot or set OPENOCD_ROOT.'
}
$openOcdRoot = (Resolve-Path -LiteralPath $OpenOcdRoot -ErrorAction Stop).Path
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

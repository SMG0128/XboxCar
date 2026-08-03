param(
  [Parameter(Mandatory = $false)]
  [string]$ElfPath = (Join-Path $PSScriptRoot 'build\Debug\stm32workspace.elf'),

  [Parameter(Mandatory = $false)]
  [ValidateRange(1, 20)]
  [int]$Attempts = 5,

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
$programCommand = "program {$elfFullPath} verify reset exit"

for ($attempt = 1; $attempt -le $Attempts; $attempt++) {
  Write-Host "Flash attempt $attempt of $Attempts..."
  & $openOcdExe `
    -s $openOcdScripts `
    -f $openOcdConfig `
    -c $programCommand

  if ($LASTEXITCODE -eq 0) {
    Write-Host 'Flash, verification, and automatic reset completed successfully.'
    exit 0
  }

  if ($attempt -lt $Attempts) {
    Start-Sleep -Seconds 2
  }
}

throw "OpenOCD failed after $Attempts attempts. The four-wire probe has no NRST signal and the running target is not accepting an SWD connection."

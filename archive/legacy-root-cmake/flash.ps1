param(
  [Parameter(Mandatory = $false)]
  [string]$ElfPath = (Join-Path $PSScriptRoot 'cmake-build-cmake4stm32\stm32workspace.elf'),

  [Parameter(Mandatory = $false)]
  [ValidateRange(1, 20)]
  [int]$Attempts = 5
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

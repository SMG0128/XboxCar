[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$project = $root
$configure = Join-Path $root 'configure.ps1'
$toolchainBin = 'C:\ST\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin'
$cmake = (Get-Command cmake -ErrorAction Stop).Source
$sizeTool = Join-Path $toolchainBin 'arm-none-eabi-size.exe'

function Invoke-External {
    param([string]$File, [string[]]$Arguments)
    Write-Host ("`n> {0} {1}" -f $File, ($Arguments -join ' '))
    & $File @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code ${LASTEXITCODE}: $File"
    }
}

if (-not (Test-Path -LiteralPath $project -PathType Container)) {
    throw "Actual project directory not found: $project"
}
if (-not (Test-Path -LiteralPath $sizeTool -PathType Leaf)) {
    throw "ARM size tool not found: $sizeTool"
}

& $configure
if ($LASTEXITCODE -ne 0) {
    throw "Configuration failed with exit code $LASTEXITCODE"
}

$artifacts = @()
foreach ($configuration in @('Debug', 'Release')) {
    $buildDir = Join-Path $project ("build\{0}" -f $configuration)
    Invoke-External $cmake @('--build', $buildDir, '--parallel')

    $base = Join-Path $buildDir 'stm32workspace'
    foreach ($artifact in @("${base}.elf", "${base}.bin", "${base}.hex")) {
        if (-not (Test-Path -LiteralPath $artifact -PathType Leaf)) {
            throw "Build succeeded but artifact was not found: $artifact"
        }
        $item = Get-Item -LiteralPath $artifact
        $artifacts += [PSCustomObject]@{
            Configuration = $configuration
            Type = $item.Extension.TrimStart('.').ToUpperInvariant()
            Path = $item.FullName
            Bytes = $item.Length
        }
    }
}

Write-Host "`nFirmware artifacts:"
$artifacts | Format-Table -AutoSize
Write-Host "`nARM size:"
& $sizeTool (Join-Path $project 'build\Debug\stm32workspace.elf') (Join-Path $project 'build\Release\stm32workspace.elf')
if ($LASTEXITCODE -ne 0) {
    throw "arm-none-eabi-size failed with exit code $LASTEXITCODE"
}
exit 0

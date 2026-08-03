[CmdletBinding()]
param(
    [ValidateSet('All', 'Firmware', 'Tests')]
    [string]$Scope = 'All'
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$project = Join-Path $root 'XboxCar_STM32_Display'
$toolchainBin = 'C:\ST\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin'

function Invoke-External {
    param([string]$File, [string[]]$Arguments)
    Write-Host ("`n> {0} {1}" -f $File, ($Arguments -join ' '))
    & $File @Arguments
    if ($LASTEXITCODE -ne 0) { throw "Command failed with exit code ${LASTEXITCODE}: $File" }
}

function Find-VcVars {
    $existingCl = Get-Command cl.exe -ErrorAction SilentlyContinue
    if ($existingCl) { return $null }
    $roots = @('C:\Program Files (x86)\Microsoft Visual Studio', 'C:\Program Files\Microsoft Visual Studio')
    foreach ($searchRoot in $roots) {
        if (Test-Path -LiteralPath $searchRoot) {
            $candidate = Get-ChildItem -LiteralPath $searchRoot -Recurse -Filter vcvars64.bat -File -ErrorAction SilentlyContinue | Select-Object -First 1
            if ($candidate) { return $candidate.FullName }
        }
    }
    return $null
}

function Invoke-HostExternal {
    param([string]$File, [string[]]$Arguments, [string]$VcVars)
    $quotedArgs = ($Arguments | ForEach-Object { '"' + ($_ -replace '"', '\"') + '"' }) -join ' '
    if ($VcVars) {
        $command = 'call "{0}" && "{1}" {2}' -f $VcVars, $File, $quotedArgs
        Write-Host ("`n> cmd /c {0}" -f $command)
        & cmd.exe /d /c $command
    } else {
        Write-Host ("`n> {0} {1}" -f $File, ($Arguments -join ' '))
        & $File @Arguments
    }
    if ($LASTEXITCODE -ne 0) { throw "Host command failed with exit code ${LASTEXITCODE}: $File" }
}

if (-not (Test-Path -LiteralPath $project -PathType Container)) { throw "Actual project directory not found: $project" }
$cmake = (Get-Command cmake -ErrorAction Stop).Source
$requiredTools = @('arm-none-eabi-gcc.exe', 'arm-none-eabi-g++.exe', 'arm-none-eabi-objcopy.exe', 'arm-none-eabi-size.exe')
foreach ($name in $requiredTools) {
    $path = Join-Path $toolchainBin $name
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Required ARM tool not found: $path" }
}

$vcvars = Find-VcVars
if (-not $vcvars -and -not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
    throw 'No MSVC host compiler environment found. Install Visual Studio Build Tools or expose cl.exe.'
}

Push-Location $project
try {
    if ($Scope -in @('All', 'Firmware')) {
        Invoke-External $cmake @('--preset', 'Debug', "-DARM_GCC_ROOT=$toolchainBin")
        Invoke-External $cmake @('--preset', 'Release', "-DARM_GCC_ROOT=$toolchainBin")
    }
    if ($Scope -in @('All', 'Tests')) {
        $testsSource = Join-Path $project 'tests'
        $testsBuild = Join-Path $project 'build\tests'
        Invoke-HostExternal $cmake @('-S', $testsSource, '-B', $testsBuild, '--fresh', '-G', 'Ninja', '-DCMAKE_C_COMPILER=cl', '-DCMAKE_CXX_COMPILER=cl', '-DCMAKE_BUILD_TYPE=Debug') $vcvars
    }
}
finally { Pop-Location }
Write-Host "`nConfiguration completed for $project"
exit 0
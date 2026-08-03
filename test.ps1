[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$project = Join-Path $root 'XboxCar_STM32_Display'
$testsSource = Join-Path $project 'tests'
$testsBuild = Join-Path $project 'build\tests'
$cmake = (Get-Command cmake -ErrorAction Stop).Source
$ctest = Join-Path (Split-Path -Parent $cmake) 'ctest.exe'

function Find-VcVars {
    if (Get-Command cl.exe -ErrorAction SilentlyContinue) { return $null }
    foreach ($searchRoot in @('C:\Program Files (x86)\Microsoft Visual Studio', 'C:\Program Files\Microsoft Visual Studio')) {
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

if (-not (Test-Path -LiteralPath $testsSource -PathType Container)) { throw "Test source directory not found: $testsSource" }
$vcvars = Find-VcVars
if (-not $vcvars -and -not (Get-Command cl.exe -ErrorAction SilentlyContinue)) { throw 'No MSVC host compiler environment found.' }

Invoke-HostExternal $cmake @('-S', $testsSource, '-B', $testsBuild, '--fresh', '-G', 'Ninja', '-DCMAKE_C_COMPILER=cl', '-DCMAKE_CXX_COMPILER=cl', '-DCMAKE_BUILD_TYPE=Debug') $vcvars
Invoke-HostExternal $cmake @('--build', $testsBuild, '--parallel') $vcvars

$ctestArgs = @('--test-dir', $testsBuild, '--output-on-failure')
$quotedArgs = ($ctestArgs | ForEach-Object { '"' + ($_ -replace '"', '\"') + '"' }) -join ' '
if ($vcvars) {
    $ctestCommand = 'call "{0}" && "{1}" {2}' -f $vcvars, $ctest, $quotedArgs
    Write-Host ("`n> cmd /c {0}" -f $ctestCommand)
    $ctestOutput = @(& cmd.exe /d /c $ctestCommand 2>&1)
} else {
    Write-Host ("`n> {0} {1}" -f $ctest, ($ctestArgs -join ' '))
    $ctestOutput = @(& $ctest @ctestArgs 2>&1)
}
$ctestExit = $LASTEXITCODE
$ctestOutput | ForEach-Object { Write-Host $_ }

$summaryText = $ctestOutput -join "`n"
$summary = [regex]::Match($summaryText, '([0-9]+)% tests passed, ([0-9]+) tests failed out of ([0-9]+)')
$passed = 0; $failed = 0; $total = 0
if ($summary.Success) {
    $failed = [int]$summary.Groups[2].Value
    $total = [int]$summary.Groups[3].Value
    $passed = $total - $failed
} else {
    $passed = @($ctestOutput | Select-String -Pattern 'Passed').Count
    $failed = @($ctestOutput | Select-String -Pattern 'Failed|Not Run').Count
    $total = $passed + $failed
}
Write-Host "`nTest summary: passed=$passed failed=$failed total=$total"
if ($ctestExit -ne 0) { exit $ctestExit }
if ($total -eq 0) { throw 'ctest discovered no tests; refusing to report success.' }
exit 0
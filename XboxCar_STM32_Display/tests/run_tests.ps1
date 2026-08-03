$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$testTemp = Join-Path $projectRoot 'build\host-test-tmp'
New-Item -ItemType Directory -Force -Path $testTemp | Out-Null

$bash = 'C:\msys64\usr\bin\bash.exe'
$cygpath = 'C:\msys64\usr\bin\cygpath.exe'
if (!(Test-Path -LiteralPath $bash) -or !(Test-Path -LiteralPath $cygpath)) {
  throw 'MSYS2 bash/cygpath was not found under C:\msys64.'
}

$msysRoot = (& $cygpath -u $projectRoot).Trim()
$msysTemp = (& $cygpath -u $testTemp).Trim()
$command = @"
export PATH=/ucrt64/bin:/usr/bin:`$PATH
export TMP='$msysTemp'
export TEMP=`$TMP
cd '$msysRoot'
COMMON='-std=c11 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Werror -DAPP_FEATURE_ULTRASONIC=1 -IApp/Inc -Itests'
APP='App/Src/xbox_protocol.c App/Src/motor.c App/Src/ultrasonic.c App/Src/safety_controller.c App/Src/diagnostics.c App/Src/debug_log.c App/Src/control_report.c App/Src/control_system.c'
gcc `$COMMON `$APP \
  tests/test_framework.c tests/test_helpers.c tests/test_protocol.c \
  tests/test_motor.c tests/test_ultrasonic.c tests/test_safety.c \
  tests/test_control.c tests/test_system.c tests/test_report.c tests/test_main.c \
  -o build/host-xboxcar-tests.exe
./build/host-xboxcar-tests.exe
gcc `$COMMON `$APP tests/test_helpers.c tests/scenario_sim.c \
  -o build/host-xboxcar-scenario.exe
./build/host-xboxcar-scenario.exe
"@

& $bash -lc $command
if ($LASTEXITCODE -ne 0) {
  throw 'Host tests failed.'
}

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
gcc -std=c11 -Wall -Wextra -Werror -DUNIT_TEST \
  -Itests/fakes -ICore/Inc \
  tests/test_control.c \
  Core/Src/control_mixer.c \
  Core/Src/control_protocol.c \
  Core/Src/control_system.c \
  -o tests/test_control.exe
./tests/test_control.exe
"@

& $bash -lc $command
if ($LASTEXITCODE -ne 0) {
  throw 'Host tests failed.'
}

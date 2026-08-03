# XboxCar STM32 project entry point

## Single firmware project root

The only real XboxCar STM32 firmware project is:

`D:\XboxCar\STM32\XboxCar_STM32_Display`

The outer `D:\XboxCar\STM32` directory is only a unified entry point and archive. It is not a second firmware project. The incomplete outer CMake, tests, presets, toolchain, flash scripts, and old README are in `archive\legacy-root-cmake`. The outer legacy `App`, `Core`, `Drivers`, and `tests` directories must not be used for development or rebuilt as another project.

## Four unified commands

Run these from `D:\XboxCar\STM32`:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\configure.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\build.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\test.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\clean.ps1
```

`build.ps1` configures and builds Debug and Release. `test.ps1` configures, builds, and runs all host tests.

## Toolchain and build directories

ARM GCC bin directory:

`C:\ST\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin`

All active build output is under the single project:

- Debug firmware: `XboxCar_STM32_Display\build\Debug`
- Release firmware: `XboxCar_STM32_Display\build\Release`
- Host tests: `XboxCar_STM32_Display\build\tests`

Expected firmware artifacts:

- `XboxCar_STM32_Display\build\Debug\stm32workspace.elf`
- `XboxCar_STM32_Display\build\Debug\stm32workspace.bin`
- `XboxCar_STM32_Display\build\Debug\stm32workspace.hex`
- `XboxCar_STM32_Display\build\Release\stm32workspace.elf`
- `XboxCar_STM32_Display\build\Release\stm32workspace.bin`
- `XboxCar_STM32_Display\build\Release\stm32workspace.hex`

## OpenOCD example

After `build.ps1`, pass the OpenOCD installation root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\XboxCar_STM32_Display\flash.ps1 `
  -ElfPath .\XboxCar_STM32_Display\build\Debug\stm32workspace.elf `
  -OpenOcdRoot 'D:\openocd-20260302\OpenOCD-20260302-0.12.0'
```

Alternatively set `OPENOCD_ROOT`. Use `recover-and-flash.ps1` when the probe has no NRST connection.

## Source locations

Add firmware source, tests, toolchain files, and build configuration only under:

`XboxCar_STM32_Display\App`, `Board`, `Core`, `Drivers`, `tests`, and `cmake`.

Do not recreate or maintain duplicate project directories under:

`D:\XboxCar\STM32\App`, `D:\XboxCar\STM32\Core`, `D:\XboxCar\STM32\Drivers`, or `D:\XboxCar\STM32\tests`.
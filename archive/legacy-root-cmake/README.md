# Legacy root CMake and test files

Archive date: 2026-08-03

This directory contains the old CMake, preset, toolchain, test, flash, and README files that were moved from `D:\XboxCar\STM32`. They belonged to an incomplete duplicate project. The old root CMake referenced application modules that did not exist in the outer tree, and the old root tests referenced missing test sources.

The only real XboxCar STM32 firmware project is:

`D:\XboxCar\STM32\XboxCar_STM32_Display`

The outer directory now contains only unified entry scripts, documentation, and this archive. Do not develop in this archive or recreate a second project under the old outer `App`, `Core`, `Drivers`, or `tests` directories.

The original files were moved, not deleted. The outer source directories were intentionally left in place because the audit found same-relative-path files with different contents; they remain preserved legacy material until a separate manual review decides what to do with them.
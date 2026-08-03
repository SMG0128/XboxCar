set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Locate Arm GNU Toolchain. CLion does not always inherit STM32CubeCLT's PATH.
set(ARM_GCC_ROOT "" CACHE PATH
    "Arm GNU Toolchain root directory (or its bin directory)")

set(ARM_GCC_HINTS)
if(ARM_GCC_ROOT)
    list(APPEND ARM_GCC_HINTS "${ARM_GCC_ROOT}" "${ARM_GCC_ROOT}/bin")
endif()
if(DEFINED ENV{STM32CUBECLT_PATH})
    list(APPEND ARM_GCC_HINTS
        "$ENV{STM32CUBECLT_PATH}/GNU-tools-for-STM32/bin")
endif()
if(WIN32)
    file(GLOB STM32CUBECLT_GCC_DIRS
        "C:/ST/STM32CubeCLT_*/GNU-tools-for-STM32/bin")
    list(SORT STM32CUBECLT_GCC_DIRS COMPARE NATURAL ORDER DESCENDING)
    list(APPEND ARM_GCC_HINTS ${STM32CUBECLT_GCC_DIRS})
endif()

find_program(ARM_GCC NAMES arm-none-eabi-gcc HINTS ${ARM_GCC_HINTS})
if(NOT ARM_GCC)
    message(FATAL_ERROR
        "arm-none-eabi-gcc was not found. Add it to PATH or configure "
        "-DARM_GCC_ROOT=<toolchain-or-bin-directory>.")
endif()

get_filename_component(ARM_GCC_BIN_DIR "${ARM_GCC}" DIRECTORY)
find_program(ARM_GXX NAMES arm-none-eabi-g++ HINTS "${ARM_GCC_BIN_DIR}"
    NO_DEFAULT_PATH REQUIRED)
find_program(ARM_OBJCOPY NAMES arm-none-eabi-objcopy
    HINTS "${ARM_GCC_BIN_DIR}" NO_DEFAULT_PATH REQUIRED)
find_program(ARM_SIZE NAMES arm-none-eabi-size HINTS "${ARM_GCC_BIN_DIR}"
    NO_DEFAULT_PATH REQUIRED)

set(CMAKE_C_COMPILER                "${ARM_GCC}")
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              "${ARM_GXX}")
set(CMAKE_LINKER                    "${ARM_GXX}")
set(CMAKE_OBJCOPY                   "${ARM_OBJCOPY}")
set(CMAKE_SIZE                      "${ARM_SIZE}")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m3 ")

set(CMAKE_C_FLAGS_INIT
    "${TARGET_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")
set(CMAKE_ASM_FLAGS_INIT
    "${TARGET_FLAGS} -x assembler-with-cpp -MMD -MP")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS_INIT
    "${CMAKE_C_FLAGS_INIT} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS_INIT
    "${TARGET_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F103xx_FLASH.ld\" \
--specs=nano.specs -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections \
-Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")

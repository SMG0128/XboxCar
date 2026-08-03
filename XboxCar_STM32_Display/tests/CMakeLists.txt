cmake_minimum_required(VERSION 3.22)

# Host test build for the portable application layer.
#
# Standalone on purpose: configuring this directory uses the default host
# compiler, whereas the firmware build at the repository root forces the
# arm-none-eabi toolchain file. Keeping them separate means neither has to
# know about the other.
#
#   cmake -S tests -B build/tests
#   cmake --build build/tests
#   ./build/tests/xboxcar_tests
#
# Only App/ sources are compiled here. Anything that touches the STM32 HAL
# lives in Board/ and is deliberately out of scope: if a business module ever
# needs a HAL header, this build stops compiling, which is the point.

project(xboxcar_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../App)

add_executable(xboxcar_tests
    ${APP_DIR}/Src/xbox_protocol.c
    ${APP_DIR}/Src/motor.c
    ${APP_DIR}/Src/ultrasonic.c
    ${APP_DIR}/Src/safety_controller.c
    ${APP_DIR}/Src/diagnostics.c
    ${APP_DIR}/Src/debug_log.c
    ${APP_DIR}/Src/control_report.c
    ${APP_DIR}/Src/control_system.c

    test_framework.c
    test_helpers.c
    test_protocol.c
    test_motor.c
    test_ultrasonic.c
    test_safety.c
    test_control.c
    test_system.c
    test_report.c
    test_main.c
)

target_include_directories(xboxcar_tests PRIVATE
    ${APP_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Exercise the obstacle chain on the host even though unsafe 5 V ECHO inputs
# keep the feature disabled in the default firmware image.
target_compile_definitions(xboxcar_tests PRIVATE APP_FEATURE_ULTRASONIC=1)

if(MSVC)
    target_compile_options(xboxcar_tests PRIVATE /W4)
else()
    target_compile_options(xboxcar_tests PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion)
endif()

# The scenario simulation shares the same modules and prints a timeline.
add_executable(xboxcar_scenario
    ${APP_DIR}/Src/xbox_protocol.c
    ${APP_DIR}/Src/motor.c
    ${APP_DIR}/Src/ultrasonic.c
    ${APP_DIR}/Src/safety_controller.c
    ${APP_DIR}/Src/diagnostics.c
    ${APP_DIR}/Src/control_system.c

    test_helpers.c
    scenario_sim.c
)

target_include_directories(xboxcar_scenario PRIVATE
    ${APP_DIR}/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(xboxcar_scenario PRIVATE APP_FEATURE_ULTRASONIC=1)

enable_testing()
add_test(NAME unit COMMAND xboxcar_tests)
add_test(NAME scenario COMMAND xboxcar_scenario)

#ifndef FW_VERSION_H
#define FW_VERSION_H

/*
 * Firmware, protocol and configuration identity.
 *
 * Kept free of build-system generated values so that a plain build stays
 * reproducible. The optional git description is injected by CMake when a git
 * work tree is available and falls back to "nogit" otherwise.
 */

#include "app_config.h"

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 2
#define FW_VERSION_PATCH 0

/* Matches config/ESP32_STM32_PROTOCOL.md and the ESP32 emitter. */
#define PROTOCOL_VERSION 1

#define FW_STRINGIFY_(x) #x
#define FW_STRINGIFY(x) FW_STRINGIFY_(x)

#define FW_VERSION_STRING           \
  FW_STRINGIFY(FW_VERSION_MAJOR)    \
  "." FW_STRINGIFY(FW_VERSION_MINOR) \
  "." FW_STRINGIFY(FW_VERSION_PATCH)

#ifdef DEBUG
#define FW_BUILD_TYPE "Debug"
#else
#define FW_BUILD_TYPE "Release"
#endif

#define FW_TARGET_MCU "STM32F103C8T6"

#ifndef FW_GIT_DESCRIBE
#define FW_GIT_DESCRIBE "nogit"
#endif

#define FW_BUILD_DATE __DATE__
#define FW_BUILD_TIME __TIME__

#endif /* FW_VERSION_H */

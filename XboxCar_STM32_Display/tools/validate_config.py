#!/usr/bin/env python3
"""Validate the STM32 board/config/build manifests without modifying files."""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
errors: list[str] = []


def read(relative: str) -> str:
    path = ROOT / relative
    if not path.is_file():
        errors.append(f"missing required file: {relative}")
        return ""
    return path.read_text(encoding="utf-8")


pins_text = read("Board/Inc/board_pins.h")
pin_rows = re.findall(
    r"^BOARD_PIN\(([A-Z0-9_]+),\s*([AB]),\s*(\d+)\)", pins_text, re.MULTILINE
)
physical: dict[tuple[str, int], str] = {}
for name, port, number_text in pin_rows:
    key = (port, int(number_text))
    if key in physical:
        errors.append(f"pin P{port}{number_text} assigned to {physical[key]} and {name}")
    physical[key] = name

required_pins = {
    "ESP_UART_TX": ("B", 6),
    "ESP_UART_RX": ("B", 7),
    "OLED_SCL": ("B", 8),
    "OLED_SDA": ("B", 9),
    "SWDIO": ("A", 13),
    "SWCLK": ("A", 14),
}
declared = {name: (port, int(number)) for name, port, number in pin_rows}
for name, expected in required_pins.items():
    if declared.get(name) != expected:
        errors.append(f"{name} must be P{expected[0]}{expected[1]}")

ioc = read("stm32workspace.ioc")
for token in (
    "Mcu.IP4=USART1",
    "PB6.Signal=USART1_TX",
    "PB7.Signal=USART1_RX",
    "PB8.Signal=I2C1_SCL",
    "PB9.Signal=I2C1_SDA",
    "SYS.Debug=Serial_Wire",
    "RCC.TimSysFreq_Value=8000000",
):
    if token not in ioc:
        errors.append(f"CubeMX configuration missing: {token}")

config = read("App/Inc/app_config.h")
for pattern, description in (
    (r"#define APP_FEATURE_ULTRASONIC 1\b", "ultrasonic enabled switch"),
    (r"#define ULTRASONIC_ENABLED_MASK 0x0BU\b", "three-channel ultrasonic mask"),
    (r"#define APP_SYSCLK_HZ 8000000UL\b", "8 MHz application clock"),
    (r"#define SOFT_PWM_RESOLUTION 100U\b", "100-step PWM"),
    (r"#define SOFT_PWM_FREQUENCY_HZ 200U\b", "200 Hz PWM carrier"),
):
    if re.search(pattern, config) is None:
        errors.append(f"app_config missing {description}")

project_sources = [
    "App/Src/xbox_protocol.c",
    "App/Src/motor.c",
    "App/Src/ultrasonic.c",
    "App/Src/safety_controller.c",
    "App/Src/diagnostics.c",
    "App/Src/control_system.c",
    "Board/Src/board_runtime.c",
]
cmake = read("CMakeLists.txt").replace("\\", "/")
iar = read("EWARM/stm32workspace.ewp").replace("\\", "/")
for source in project_sources:
    if source not in cmake:
        errors.append(f"CMake source list missing {source}")
    if source not in iar:
        errors.append(f"IAR source list missing {source}")

for include in ("App/Inc", "Board/Inc"):
    if include not in cmake:
        errors.append(f"CMake include list missing {include}")
    if include not in iar:
        errors.append(f"IAR include list missing {include}")

if "Core/Src/xbox_protocol.c" in cmake or "Core/Src/xbox_protocol.c" in iar:
    errors.append("legacy Core xbox_protocol.c must not be in an active build")

for folder in (ROOT / "App", ROOT / "Board"):
    for path in folder.rglob("*.[ch]"):
        text = path.read_text(encoding="utf-8")
        if re.search(r"\b(malloc|calloc|realloc|free)\s*\(", text):
            errors.append(f"dynamic allocation found in {path.relative_to(ROOT)}")
        if "FreeRTOS" in text:
            errors.append(f"RTOS dependency found in {path.relative_to(ROOT)}")

if errors:
    print("configuration validation: FAIL")
    for error in errors:
        print(f"  - {error}")
    sys.exit(1)

print(f"configuration validation: PASS ({len(pin_rows)} unique PCB pins)")
print(f"CMake/IAR parity: PASS ({len(project_sources)} application sources)")
print("clock/UART/I2C/PWM/feature checks: PASS")

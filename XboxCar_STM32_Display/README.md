# XboxCar STM32 控制固件

目标为 STM32F103C8T6（LQFP48、64 KiB Flash、20 KiB RAM），使用
STM32CubeF1 HAL 1.8.7、HSI 8 MHz、CMake/Ninja/Arm GCC，同时保留 CubeMX
`.ioc` 与 IAR 工程。固件不使用 RTOS 和动态内存。

ESP32-S3 已完成手柄死区、摇杆处理和差速混合；STM32 只接收最终
`LEFT/RIGHT`，依次执行协议与序号校验、安全状态仲裁、可选避障限速、10 ms
速度斜坡和四路电机输出。

## 硬件接口

| 功能 | STM32 引脚 |
| --- | --- |
| ESP32 控制串口 | USART1 remap：PB6 TX、PB7 RX，115200 8N1 |
| OLED | I2C1 remap：PB8 SCL、PB9 SDA，正常 3V3/GND 供电 |
| 左前电机 | PA1 PWM、PA2 IN1、PA3 IN2 |
| 左后电机 | PA6 PWM、PA4 IN1、PA5 IN2 |
| 右前电机 | PB3 PWM、PA12 IN1、PA15 IN2 |
| 右后电机 | PB15 PWM、PA8 IN1、PA9 IN2 |

PB3、PA15 需要关闭 JTAG-DP；固件保留 PA13/PA14 的 SWD。两片 TB6612 的
VM/STBY 和 HC-SR04 的 5 V ECHO 风险必须先按
`config/PCB_HARDWARE_WARNINGS.md` 处理。

## 控制与安全

固定协议保持不变：

```text
$XC,<CMD>,<LEFT>,<RIGHT>,<SEQ>,<CRC>\r\n
$XC,0001,+0800,+0800,0025,1D\r\n
```

- 上电默认关闭全部电机，首个完整合法控制帧前不输出；
- 只有格式、数值、CRC 和序号全部可接受的帧才刷新 300 ms 通信看门狗；
- 重复帧不更新控制、不刷新超时、也不计入急停恢复；
- 急停和错误命令锁定停车，连续 3 帧合法非急停帧后才解除；
- 内部故障、锁定急停、通信超时均立即归零，不等待速度斜坡；
- 普通换向必须先减速到零并保持 2 个控制周期；
- 超声波默认关闭，只有四路 ECHO 完成电平转换后才能启用；
- OLED 只读统一诊断状态，每次仅刷新 128 字节的一页，不参与控制。

详细规则见 `config/ESP32_STM32_PROTOCOL.md`、
`config/SAFETY_CONTROL_DESIGN.md` 和 `config/ULTRASONIC_DESIGN.md`。

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

输出分别为：

```text
build/Debug/stm32workspace.elf
build/Release/stm32workspace.elf
```

主机测试使用独立构建目录，不会引入 STM32 HAL：

```powershell
cmake -S tests -B build/tests -G Ninja
cmake --build build/tests
ctest --test-dir build/tests --output-on-failure
```

`xboxcar_tests` 覆盖协议、序号、电机、急停、超时、超声波、避障和系统场景；
`xboxcar_scenario` 输出完整的虚拟行驶时间线。它们是主机模拟，不代表实车验证。

## 目录

| 目录/文件 | 职责 |
| --- | --- |
| `App/Inc`, `App/Src` | 无 HAL 依赖的协议、状态机、电机、超声波、安全和诊断逻辑 |
| `Board/Inc`, `Board/Src` | PCB 引脚、USART1、TIM4 软件 PWM、GPIO、调度和 OLED 适配 |
| `Core` | CubeMX 入口、中断、HAL MSP 与 SSD1306 驱动 |
| `tests` | 主机单元测试和系统场景 |
| `config` | 协议、硬件风险、架构、安全、超声波和上车指南 |
| `stm32workspace.ioc` | CubeMX 引脚与外设源文件 |
| `EWARM/stm32workspace.ewp` | 与 CMake 同步的 IAR 源文件和包含路径 |

烧录目标 `flash` 与无 NRST 的人工复位恢复目标 `recover-flash` 沿用现有脚本。
在 VM/STBY、共地、电机方向和 ECHO 电平未确认前，不要带电机落地测试。

This directory is the sole firmware project root for XboxCar STM32.


该目录是 XboxCar STM32 的唯一固件工程根目录。

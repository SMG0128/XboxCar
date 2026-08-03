# XboxCar PCB 接口反推报告

## MCU 与电机驱动

目标 MCU 为 STM32F103C8T6（LQFP48），电机驱动为左右各一片 TB6612。

| 电机 | TB6612 通道 | PWM | IN1 | IN2 |
| --- | --- | --- | --- | --- |
| 左前 | 左侧 B | PA1 | PA2 | PA3 |
| 左后 | 左侧 A | PA6 | PA4 | PA5 |
| 右前 | 右侧 A | PB3 | PA12 | PA15 |
| 右后 | 右侧 B | PB15 | PA8 | PA9 |

每个电机的安装方向修正在 `App/Inc/app_config.h` 的
`MOTOR_*_INVERTED` 中集中配置。默认右侧两电机反向。

## 通信与传感器

| 接口 | 引脚 |
| --- | --- |
| USART1 remap TX/RX | PB6 / PB7 |
| I2C1 remap SCL/SDA | PB8 / PB9 |
| 超声波 FL TRIG/ECHO | PB11 / PB10 |
| 超声波 FR TRIG/ECHO | PB13 / PB12 |
| 超声波 BL TRIG/ECHO | PB0 / PB1 |
| 超声波 BR TRIG/ECHO | PA7 / PB14 |

USART1 使用 115200 baud、8N1。PB8/PB9 保留给 I2C1。旧显示工程曾把
PB6/PB7 当作 OLED GND/VCC，并用 PB10/PB11 运行 USART3；这与 PCB 电机/超声波
接口冲突，本版已经改回 PCB 定义。OLED 必须从正常的 GND/3V3 供电。

## 必须人工修正的 PCB 问题

- 左右 TB6612 的 VM 未接入 PCB 电源网络。必须飞线接入电机电源或在下一版 PCB 修正。
- 左右 TB6612 的 STBY 未接 STM32。必须在硬件上拉到 3V3，或下一版接入 GPIO。
- 程序没有虚构 STBY GPIO，也不能检测 VM 是否已经接好。
- PB3、PA15 默认属于 JTAG；固件关闭 JTAG-DP、保留 SWD。
- PA1 与 PB3 不能同时成为两个独立 TIM2_CH2。四路 PWM 因此使用 TIM4 基准中断的软件 PWM。

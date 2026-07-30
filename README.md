# XboxCar STM32 四电机底层

本工程基于现有 STM32F103C8T6 CubeMX/HAL 工程增量开发，保留 CMake、IAR、
OpenOCD 和 SSD1306 支持，不使用 RTOS。

## 功能

- USART1 重映射 PB6/PB7，115200 8N1，按字节中断接收；
- 256 字节无动态内存环形缓冲和 30 字节 ASCII 帧流式解析；
- 四个 TB6612 电机通道、独立方向反转配置；
- TIM4 20 kHz 基准中断、100 级、200 Hz 四路软件 PWM；
- 左右速度 10 ms / 40 步斜坡，换向先过零；
- 300 ms 通信超时立即停车；
- 锁定急停和连续 3 帧合法命令恢复；
- 四路超声波非阻塞轮询状态机框架（因 ECHO 电平风险默认关闭）；
- 可由调试器读取的 `ControlStatus` 统计与状态；
- 纯算法、协议、超时和急停主机测试源码。

## 工程识别

| 项目 | 值 |
| --- | --- |
| MCU | STM32F103C8T6，LQFP48 |
| 框架 | STM32CubeF1 HAL 1.8.7 |
| 时钟 | HSI 8 MHz |
| 构建 | CMake + Ninja + Arm GNU Toolchain |
| 启动文件 | `startup_stm32f103xb.s` |
| 链接脚本 | `STM32F103C8TX_FLASH.ld` |

## 构建

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel
```

输出为 `build/Debug/stm32workspace.elf`。

主机测试：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_tests.ps1
```

## 烧录

```powershell
cmake --build --preset Debug --target flash
```

如果 ST-Link 没有连接 NRST、正常接管失败：

```powershell
cmake --build --preset Debug --target recover-flash
```

运行恢复目标时反复短按 RESET，看到 OpenOCD 开始写入后停止，成功后再短按一次
RESET 运行。

## 文档

- `config/STM32_CONTROL_ARCHITECTURE.md`
- `config/STM32_BRINGUP_GUIDE.md`
- `config/ESP32_STM32_PROTOCOL.md`
- `config/PCB_HARDWARE_WARNINGS.md`
- `config/XboxCar_PCB_接口反推报告.md`

烧录和接电机前必须先阅读硬件警告：两片 TB6612 的 VM、STBY 均没有被 PCB 正确
连接，软件不能修复这一问题。


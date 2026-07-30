# XboxCar STM32 串口解析与 OLED 显示

本工程基于 `D:\XboxCar\stm32workspace` 创建，原工程未被修改。目标芯片为 `STM32F103C8T6`。

STM32 通过 USART3 接收 ESP32-S3 发出的 XboxCar 控制帧，校验帧格式和 XOR 后，在 128×64 SSD1306 OLED 上显示运动方向与速度。

## 接线

### ESP32-S3 与 STM32

| ESP32-S3 | STM32F103 | 用途 |
| --- | --- | --- |
| GPIO43 / UART0 TX | PB11 / USART3 RX | ESP32 向 STM32 发送控制帧 |
| GND | GND | 共地 |
| 可不接 | PB10 / USART3 TX | STM32 发送端，当前程序未使用 |

USART3 配置为 `115200 baud, 8N1`。

### OLED

按当前硬件接线：

| STM32F103 | OLED | 配置 |
| --- | --- | --- |
| PB6 | GND | GPIO 推挽输出低电平 |
| PB7 | VCC | GPIO 推挽输出高电平 |
| PB8 | SCL | 重映射 I2C1 SCL |
| PB9 | SDA | 重映射 I2C1 SDA |

I2C1 频率为 400 kHz，SSD1306 地址默认为 `0x3C`。

> PB6/PB7 通过 GPIO 给 OLED 提供地和电源是按照当前接线实现的。请确认 OLED 模块工作电流没有超过 STM32 GPIO 的允许电流；正式硬件更推荐直接连接稳定的 GND 和 3.3 V。

原始工程没有复制 I2C/UART HAL 的源文件和头文件。新工程已从本机与 `.ioc` 一致的官方 `STM32Cube_FW_F1_V1.8.7` 固件包补入这些驱动，原工程保持不变。

## OLED 显示

OLED 使用 2 倍 5×7 字体显示两行：

```text
UP:0800
RIGHT:0350
```

- 第一行显示纵向方向：`UP`、`DOWN` 或 `STOP`
- 第一行速度：`abs((LEFT + RIGHT) / 2)`
- 第二行显示转向方向：`LEFT`、`RIGHT` 或 `LR`
- 第二行速度：`abs((LEFT - RIGHT) / 2)`
- 速度范围：`0000..1000`

以下情况显示：

```text
No Xbox
```

- 上电后尚未收到合法控制帧
- 收到 `CMD=1001` Xbox 断线帧
- 连续 200 ms 没有收到合法控制帧

`CMD=0111` 紧急停车、`1000` 尚无手柄数据和 `1111` 错误帧会强制把左右速度清零。

## 串口协议

程序只接受固定 30 字节帧：

```text
$XC,<CMD>,<LEFT>,<RIGHT>,<SEQ>,<CRC>\r\n
```

示例：

```text
$XC,0001,+0800,+0800,0025,1D\r\n
```

接收器逐字节中断接收，并以 `$` 重新同步。只有以下检查全部通过才更新 OLED 状态：

1. 帧头为 `$XC`
2. 总长度和逗号位置正确
3. 行尾为单个 `CRLF`
4. `CMD` 是四位二进制数
5. `LEFT`、`RIGHT` 是带符号四位十进制数，范围 `-1000..+1000`
6. `SEQ` 是四位十进制数
7. 两位大写十六进制 XOR 与实际计算结果一致

Bluepad32 启动日志和其他非协议文本会被自动忽略。

## 主要文件

| 文件 | 作用 |
| --- | --- |
| `stm32workspace.ioc` | CubeMX 外设与引脚配置 |
| `Core/Src/main.c` | 外设初始化、协议处理和显示调度 |
| `Core/Src/xbox_protocol.c` | 串口接收、组帧、CRC、字段校验和超时状态 |
| `Core/Inc/xbox_protocol.h` | Xbox 协议接口 |
| `Core/Src/ssd1306_simple.c` | SSD1306 初始化、字体和两行显示 |
| `Core/Inc/ssd1306_simple.h` | OLED 接口 |
| `Core/Src/stm32f1xx_hal_msp.c` | PB8/PB9 I2C1 和 PB10/PB11 USART3 底层配置 |
| `Core/Src/stm32f1xx_it.c` | USART3 中断入口 |

## 使用说明

1. 用 STM32CubeMX 打开 `stm32workspace.ioc` 检查配置。
2. 使用 CLion/CMake、STM32CubeIDE 或其他兼容工具打开工程。
3. 编译并烧录后，连接 OLED 和 ESP32-S3。
4. ESP32 上运行 XboxCar 的 `Xbox_Controller_Debug` 固件。
5. Xbox 未连接时显示 `No Xbox`；连接并操作摇杆后显示方向和速度。

本次任务按要求只创建和修改工程文件，没有编译或烧录 STM32。

## CLion OpenOCD 下载并运行

工程包含共享运行配置：

```text
.run/STM32_OpenOCD_Download_Run.run.xml
```

本机已确认安装：

| 工具 | 路径 |
| --- | --- |
| CLion | `C:\Program Files\JetBrains\CLion 2026.1.3` |
| OpenOCD | `D:\openocd-20260302\OpenOCD-20260302-0.12.0\bin\openocd.exe` |
| Arm GCC | `C:\ST\STM32CubeCLT_1.22.0\GNU-tools-for-STM32\bin\arm-none-eabi-gcc.exe` |
| CMake | `C:\ST\STM32CubeCLT_1.22.0\CMake\bin\cmake.exe` |
| Ninja | `C:\ST\STM32CubeCLT_1.22.0\Ninja\bin\ninja.exe` |

CLion 操作：

1. 用 CLion 打开本工程根目录。
2. 等待 `Debug` CMake Preset 加载完成。
3. 在运行配置中选择 `STM32 OpenOCD Download & Run`。
4. 点击运行或按 `Shift+F10`：先编译，再下载、校验、复位并运行。
5. 点击调试或按 `Shift+F9`：下载后连接 GDB，可使用断点和寄存器视图。

运行配置参数：

- CMake 目标：`stm32workspace`
- OpenOCD 配置：`stlink.cfg`
- GDB 端口：3333
- Telnet 端口：4444
- 下载策略：每次下载
- 复位策略：`reset run`
- 调试器：CLion 内置 ARM GDB

`stlink.cfg` 已针对当前 ST-Link/SWD 和 STM32F103 目标配置。当前探针没有连接 NRST，因此如果正常下载无法接管正在运行的芯片，可在 CLion 的 CMake 目标中运行 `recover-flash`，并反复短按开发板 RESET，直到 OpenOCD 开始写入；成功后再短按一次 RESET 运行。

# XboxCar ESP32-S3 控制端

本分支保存 XboxCar 项目的 ESP32-S3 控制端代码与 ESP32/STM32 串口协议。

ESP32-S3 通过 Bluepad32 连接 Xbox Wireless Controller，读取左摇杆 Y 轴和右摇杆 X 轴，计算左右轮差速，经平滑加减速后，以 50 Hz 向 STM32 或电脑串口发送 ASCII 控制帧。

## 已实现功能

- Xbox 无线手柄连接、断线检测与自动安全停车
- 左摇杆 Y 轴控制前进/后退
- 右摇杆 X 轴控制转向和原地旋转
- 左右轮差速混合，输出范围 `-1000..+1000`
- 摇杆死区、连续重映射、加减速斜坡和换向过零保护
- Xbox 系统键紧急停车
- UART0 以 115200 baud、8N1、50 Hz 输出固定长度控制帧
- 帧序号与 XOR 校验
- 板载 LED 关闭示例
- 可在电脑上运行的控制算法单元测试

## 目录结构

```text
.
├── ESP_LED_Off/
│   └── ESP_LED_Off.ino
├── Xbox_Controller_Debug/
│   ├── Xbox_Controller_Debug.ino
│   ├── XboxCarControl.h
│   ├── XboxCarControl.cpp
│   └── tests/
│       └── test_xbox_car_control.cpp
├── config/
│   └── ESP32_STM32_XBOX_PROTOCOL.md
└── README.md
```

`Xbox_Controller_Debug` 是当前实际使用的控制程序。`.ino` 负责 Bluepad32 连接、定时调度和串口发送；`XboxCarControl.*` 负责摇杆归一化、差速、斜坡、动作分类、组帧和校验。

## 硬件与依赖

- ESP32-S3 开发板
- Xbox Wireless Controller
- Arduino IDE / Arduino CLI
- Bluepad32 Arduino Core
- 当前验证的 FQBN：`esp32-bluepad32:esp32:esp32s3`
- 当前验证的 Bluepad32 ESP32 Core 版本：`4.1.0`

UART0 参数：

| 项目 | 配置 |
| --- | --- |
| 波特率 | 115200 |
| 数据格式 | 8N1 |
| ESP32 TX | GPIO43 |
| ESP32 RX | GPIO44 |
| 发送周期 | 20 ms（50 Hz） |

连接 STM32 时至少连接：

```text
ESP32 GPIO43 (TX) -> STM32 UART RX
ESP32 GND         -> STM32 GND
```

开发板的 USB 转串口也可读取相同控制帧。UART0 可能同时出现 Bluepad32 启动日志，接收端应只解析以 `$XC,` 开头且格式、长度和 CRC 均合法的帧。

## 手柄操作

| 输入 | 功能 |
| --- | --- |
| 左摇杆 Y | 前进/后退 |
| 右摇杆 X | 左转/右转；油门为零时原地旋转 |
| Xbox 系统键 | 紧急停车 |

左摇杆 Y 死区为 8%，右摇杆 X 死区为 10%。死区外会从零开始连续映射，因此越过死区边界时不会发生速度突跳。

正常驾驶最大输出为 `1000`，原地旋转最大输出为 `650`。控制循环周期为 10 ms；加速每周期最多增加 25，减速每周期最多减少 40。正反方向切换时必须先减速到零。

## 串口协议

固定帧格式：

```text
$XC,<CMD>,<LEFT>,<RIGHT>,<SEQ>,<CRC>\r\n
```

示例：

```text
$XC,0001,+0800,+0800,0025,1D\r\n
```

- `CMD`：4 位二进制动作码
- `LEFT`、`RIGHT`：带符号四位十进制数，范围 `-1000..+1000`
- `SEQ`：`0000..9999` 循环序号
- `CRC`：从 `X` 到 `SEQ` 末位所有 ASCII 字节的 8 位 XOR
- 合法帧固定为 30 字节，并以单个 `CRLF` 结束

完整字段、动作码、安全规则和 STM32 解析建议见 [config/ESP32_STM32_XBOX_PROTOCOL.md](config/ESP32_STM32_XBOX_PROTOCOL.md)。

## 编译与烧录

1. 在 Arduino IDE 中安装 Bluepad32 ESP32 Core。
2. 打开 `Xbox_Controller_Debug/Xbox_Controller_Debug.ino`。
3. 选择 ESP32-S3 对应开发板和串口。
4. 编译并烧录。
5. 如果自动复位后没有运行，松开所有按键，仅短按一次 `RST/EN`，不要按 `BOOT`。

Arduino CLI 示例：

```powershell
arduino-cli compile --fqbn esp32-bluepad32:esp32:esp32s3 Xbox_Controller_Debug
arduino-cli upload -p COM4 --fqbn esp32-bluepad32:esp32:esp32s3 Xbox_Controller_Debug
```

默认 `XBOXCAR_LOG_LEVEL=0`，串口只保留协议帧。台架调试时可改为：

- `1`：增加连接、断线和安全事件
- `2`：再增加限频的摇杆和差速诊断

正式与 STM32 联调时建议保持为 `0`。

## 安全行为

以下情况立即清零左右输出，不等待斜坡：

- Xbox 手柄断线
- 按下 Xbox 系统键
- 摇杆数据越界
- 控制帧组装失败

STM32 端还应设置 200 ms 合法帧超时；超时后立即关闭电机输出。

## 验证结果

2026-07-30 在 ESP32-S3 与 Xbox Wireless Controller 实机验证：

- 固件编译和烧录成功
- 手柄连接、停车和前进指令正常
- 3.21 秒收到 160 个合法控制帧
- 实测发送频率 49.8 Hz
- CRC 错误 0
- 串口行尾为单个 `CRLF`
- C++ 控制算法单元测试全部通过

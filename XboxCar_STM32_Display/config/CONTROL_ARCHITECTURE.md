# STM32 控制架构

## 边界

ESP32 已输出混合后的 `LEFT/RIGHT`。STM32 不再做摇杆死区或差速混合；安全模块
只在需要按前后/转向方向限速时临时分解并按相同比例重组左右轮请求。

```text
USART1 byte ISR -> 256-byte ring -> ASCII parser + sequence
  -> fault/e-stop/timeout arbitration -> optional obstacle limit
  -> 10 ms motor ramp -> four electrical outputs -> TIM4 software PWM
```

`App/` 是可移植业务层，不包含 HAL。`Board/` 是唯一板级适配层，负责 PCB 引脚、
USART1 接收重启、TIM4/GPIO、10 ms 调度、传感器回调和 OLED。`Core/` 保留 CubeMX
入口与 HAL 初始化。

## 中断边界

- USART1 ISR：HAL 状态处理、单字节入环和下一字节接收；不解析字符串；
- TIM4 ISR：比较四个 duty，合并 GPIOA/GPIOB BSRR 掩码，各写一次；
- 故障异常：停止 TIM4，并直接把全部 PWM/方向脚拉低；
- CRC、序号、安全状态、斜坡、传感器状态机和显示均在主循环。

## 时钟与 PWM

- HSI 8 MHz，无 PLL，AHB/APB1/APB2 均为 8 MHz；
- TIM4 更新中断 20 kHz；
- 100 级相位分辨率，输出 PWM 200 Hz；
- PA1、PA6、PB3、PB15 是普通推挽 GPIO；
- duty 范围 0..100，满量程在整个相位周期保持高；
- 四电机独立反向配置集中在 `App/Inc/app_config.h`。

## 状态与诊断

`ControlSystem` 是唯一控制状态源，优先级为：内部故障 > 锁定急停 > 通信超时 >
超声波限制 > 普通控制 > 斜坡。`AppDebugState` 是唯一诊断快照，OLED 和调试器只读
该快照，不反向控制车辆。

OLED 使用 PB8/PB9，不能再从 PB6/PB7 取电。显示缓冲每 50 ms 最多发送一页
128 字节，避免一次 1 KiB I2C 阻塞导致 UART 环形缓冲溢出。

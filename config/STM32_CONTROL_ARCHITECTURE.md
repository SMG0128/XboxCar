# STM32 控制架构

## 工程基线

- MCU：STM32F103C8T6，64 KiB Flash、20 KiB RAM
- 库：STM32CubeF1 HAL 1.8.7，未使用 RTOS
- 系统时钟：HSI 8 MHz
- 构建：CMake 3.22+、Ninja、Arm GNU Toolchain；另保留原 IAR 工程文件

## 模块

| 模块 | 职责 |
| --- | --- |
| `board_config` / `app_config` | PCB 引脚、反向、周期和安全参数 |
| `soft_pwm` | TIM4 20 kHz 中断中的四路统一 GPIO PWM |
| `motor` | TB6612 方向、占空比、斜坡和立即停车 |
| `control_mixer` | 纯算法死区、线性差速归一化和 approach |
| `control_protocol` | UART 中断单字节入环形缓冲、ASCII 帧流式解析和统计 |
| `control_system` | 使能、300 ms 超时、锁定急停和三帧恢复 |
| `ultrasonic` | 四路轮询式非阻塞状态机，默认关闭 |
| `app` | 1 ms/10 ms 协作调度与低频 OLED 状态显示 |

数据流为：

```text
ESP32 UART -> ISR 环形缓冲 -> 帧状态机 -> 安全状态机
           -> 左右目标 -> 10 ms 斜坡 -> 四电机方向/PWM
```

中断只保存 UART 字节、重启接收，或执行一次最小化 PWM GPIO BSRR 写入；CRC、
字段解析、状态切换和显示都在主循环执行。

## 差速与死区

附件要求的纯算法保留在 `control_mixer.c`：

```text
left_raw  = throttle + steering
right_raw = throttle - steering
```

调用方先用 `ControlMixer_ApplyDeadzone()` 应用 80 死区；死区外从 0 重新线性映射
到 1000，再把结果交给 `ControlMixer_Mix()`。混合后若最大绝对值超过 1000，左右
同时按同一比例缩小，避免独立裁剪破坏转弯比例。

实际已部署 ESP32 协议直接发送经过混合的 `LEFT`/`RIGHT`，因此 STM32 不重复对它们
做死区或差速。状态中的 `throttle`、`steering` 是由左右值反算的诊断近似值。

## PWM

PCB 的 PA1 和 PB3 都涉及 TIM2_CH2 的不同映射，PB15 又是互补输出，无法可靠配置
为四个互不冲突的普通硬件 PWM。当前方案：

- TIM4 更新中断：20 kHz；
- 相位分辨率：100 级；
- 最终 PWM：200 Hz；
- 占空比：0..100；
- PA6、PA1、PB3、PB15 都是普通推挽输出；
- ISR 合并计算 GPIOA/GPIOB 的 set/reset mask，各写一次 BSRR。

## 安全状态

上电首先把所有方向和 PWM 引脚置低。收到动作码 `0000..0110` 才允许驱动。

- 合法帧超过 300 ms 中断：立即 `Motor_StopAll()`，不经过斜坡；
- `0111` 或 `1111`、数值越界、UART 严重错误：锁定急停；
- 急停后必须连续收到 3 帧合法普通控制帧才解锁；
- `1000` NoInput 和 `1001` Disconnected 立即停车但不解锁急停；
- 普通速度变化每 10 ms 最多变化 40，换向先走到零；
- `ControlStatus` 可直接在调试器查看计数和当前状态。

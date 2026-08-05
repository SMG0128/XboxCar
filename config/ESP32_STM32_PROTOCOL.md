# XboxCar ESP32 → STM32 control protocol

## Current contract

ESP32-S3 Bluepad32 sends to STM32F103C8T6 over UART 115200 8N1. GPIO43 is
ESP32 UART0 TX to STM32 USART1 RX PB7. STM32 PB6/USART1 TX is diagnostics to
USB-TTL RX only; leave USB-TTL TX disconnected. OLED is I2C1 remap PB8=SCL
and PB9=SDA. ESP32 sends every 20 ms; STM32 controls every 10 ms.

ESP32 emits v2. STM32 accepts v1 for backward compatibility, but v1 has no
operator axes and derives them from LEFT/RIGHT.

## v2 frame

Exactly 57 ASCII bytes including CRLF:

```text
$XD,<CMD>,<LEFT>,<RIGHT>,<UP_DOWN>,<LEFT_RIGHT>,<RAW_UD>,<RAW_LR>,<FLAGS>,<SEQ>,<CRC>\r\n
```

| Field | Offset | Length | Meaning |
| --- | ---: | ---: | --- |
| `$XD` | 0 | 3 | v2 header |
| `CMD` | 4 | 4 | binary action code |
| `LEFT` | 9 | 5 | mixed left speed |
| `RIGHT` | 15 | 5 | mixed right speed |
| `UP_DOWN` | 21 | 5 | forward/back command |
| `LEFT_RIGHT` | 27 | 5 | steering command |
| `RAW_UD` | 33 | 5 | raw left stick, vehicle sign |
| `RAW_LR` | 39 | 5 | raw right-stick X |
| `FLAGS` | 45 | 2 | uppercase hexadecimal flags |
| `SEQ` | 48 | 4 | `0000..9999`, wrapping |
| `CRC` | 53 | 2 | uppercase hexadecimal XOR |
| CRLF | 55 | 2 | terminator |

Signed control values are `+0000`/`-0000` in `-1000..+1000`. Raw values are
diagnostic only and are limited to `-9999..+9999`. Flags are `0x01` connected,
`0x02` has HID sample, `0x04` emergency, and `0x08` control error.

CRC XORs ASCII bytes from the `X` through the final `SEQ` digit. `$`, CRC and
CRLF are excluded.

## Commands and directions

`0000` Stop, `0001` Forward, `0010` Reverse, `0011` TurnLeft, `0100`
TurnRight, `0101` SpinLeft, `0110` SpinRight, `0111` EmergencyStop, `1000`
NoInput, `1001` Disconnected, and `1111` Error.

`UP_DOWN > 0` is forward, `< 0` reverse, `0` neutral. `LEFT_RIGHT > 0` is
right, `< 0` left, `0` neutral. Motors use validated LEFT/RIGHT; OLED and
diagnostics use the operator axes from the same validated snapshot.

## Valid examples

The CRC values below are actual XOR results:

```text
$XD,1001,+0000,+0000,+0000,+0000,+0000,+0000,00,0000,30\r\n
$XD,1000,+0000,+0000,+0000,+0000,+0000,+0000,03,0000,32\r\n
$XD,0001,+1000,+1000,+1000,+0000,-0511,+0000,03,0001,31\r\n
$XD,0010,-0500,-0500,-0500,+0000,-0256,+0000,03,0002,34\r\n
$XD,0100,+0800,+0350,+0800,+0529,+0511,+0300,03,0003,3F\r\n
$XD,0101,-0650,+0650,+0000,-1000,+0000,-0511,03,0004,35\r\n
```

## Receiver, safety and display

STM32 receives into a USART1 ring; parsing, CRC, sequence checks and state
publication run in the control loop. Noise before `$`, partial/malformed
frames, stale frames, duplicates and CRC failures are discarded as whole frames
and cannot refresh the watchdog.

The timeout is 300 ms. On timeout, disconnect, emergency or protocol error,
STM32 forces all four motor targets and outputs to zero, shows `No Xbox`, and
logs the reason. Valid frames are required before control resumes; emergency
also requires the recovery run.

OLED uses the shared `AppDebugState` and shows fixed-width lines such as:

```text
UP   :072
LEFT :035
```

or `No Xbox`. Startup logs identify MCU, UARTs, OLED and all PWM mappings.
Runtime logs include `[CTRL]`, `[AXIS]`, `[CMD]`, `[MIX]`, `[PWM1]..[PWM4]`,
protocol errors and `[SAFE]`, rate limited without blocking UART RX.

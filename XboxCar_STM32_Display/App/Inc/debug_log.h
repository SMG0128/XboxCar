#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Non-blocking diagnostic log transport.
 *
 * The XboxCar PCB breaks out exactly one UART header, so the ESP32 control link
 * and the USB-TTL console share USART1. That sharing is only safe in one
 * direction: PB7 (RX) belongs to the ESP32 alone, and PB6 (TX) may be listened
 * to by the USB-TTL adapter. Nothing in this module ever reads.
 *
 * The rule that shapes the design is that logging must not be able to stall the
 * control loop. Formatting appends into a ring here; the board layer drains a
 * bounded number of bytes per main-loop pass straight into the USART data
 * register when the transmitter is idle. There is no blocking write, no HAL
 * UART transmit call that could collide with the receive interrupt's state
 * machine, and no transmission from inside an interrupt.
 *
 * A full ring drops whole messages rather than writing a truncated line, and
 * counts what it dropped. A log that silently emits half a line is worse than
 * one that admits it fell behind.
 *
 * No STM32 HAL dependency: the host tests drain the ring into a string and
 * assert on the exact bytes.
 */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

#if defined(__GNUC__)
#define DEBUG_LOG_PRINTF_LIKE(fmt_index, first_arg)                           \
  __attribute__((format(printf, fmt_index, first_arg)))
#else
#define DEBUG_LOG_PRINTF_LIKE(fmt_index, first_arg)
#endif

/* Clears the ring and the drop counters. */
void DebugLog_Init(void);

/*
 * Appends one formatted line. The caller supplies the trailing "\r\n"; nothing
 * is added implicitly, so a caller can build a line from several calls.
 *
 * A message that does not fit in the free space is discarded whole.
 */
void DebugLog_Printf(const char *format, ...) DEBUG_LOG_PRINTF_LIKE(1, 2);

/* Same, for a caller that already has a va_list. */
void DebugLog_VPrintf(const char *format, va_list args);

/* Appends a NUL-terminated string verbatim, with the same all-or-nothing rule. */
void DebugLog_Puts(const char *text);

/*
 * Pops one pending byte. Returns false when the ring is empty. The board layer
 * calls this in a bounded loop; the host tests call it to reassemble lines.
 */
bool DebugLog_PopByte(uint8_t *byte);

/* Bytes waiting to be transmitted. */
uint16_t DebugLog_Pending(void);

/* Messages discarded because the ring was full, cumulative. */
uint32_t DebugLog_DroppedMessages(void);

/*
 * Globally mutes the transport. Used by the Release build and by any host test
 * that wants the formatting path exercised without the byte traffic.
 */
void DebugLog_SetEnabled(bool enabled);
bool DebugLog_IsEnabled(void);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_LOG_H */

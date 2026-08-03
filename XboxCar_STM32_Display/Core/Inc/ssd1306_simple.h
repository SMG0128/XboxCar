#ifndef SSD1306_SIMPLE_H
#define SSD1306_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Minimal SSD1306 driver for the PCB's 4-pin I2C header (I2C1 remapped to
 * PB8/PB9). 128x64, one 5x7 font drawn at 2x.
 *
 * Rendering and transmission are separate on purpose. Rendering fills a RAM
 * buffer and costs nothing on the wire; transmission pushes one 128-byte page
 * per call, which bounds the blocking I2C write to roughly 3 ms so the panel can
 * never make a control period late.
 *
 * Only pages whose bytes actually changed are transmitted. A parked stick
 * therefore produces no I2C traffic, and a changed speed produces the two or
 * three pages the digits occupy rather than a whole frame.
 */

#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define SSD1306_PAGE_COUNT 8U

/* Result of one flush attempt. */
typedef enum {
  SSD1306_FLUSH_IDLE = 0,  /* nothing dirty */
  SSD1306_FLUSH_SENT = 1,  /* one page transmitted */
  SSD1306_FLUSH_FAILED = 2 /* I2C rejected the transfer */
} SSD1306FlushResult;

bool SSD1306_Init(I2C_HandleTypeDef *i2c);

/*
 * Renders the "no usable controller" screen. Idempotent: re-rendering the same
 * screen marks nothing dirty.
 */
bool SSD1306_RenderNoXbox(void);

/*
 * Renders the two fixed-width status lines. Either may be NULL or empty, which
 * draws a blank line.
 */
bool SSD1306_RenderLines(const char *line1, const char *line2);

/* Transmits at most one dirty page. */
SSD1306FlushResult SSD1306_FlushDirtyPage(void);

/* True while any page still differs from what the panel is showing. */
bool SSD1306_HasDirtyPages(void);

/* Bit per page, for the host-side format tests and for diagnostics. */
uint8_t SSD1306_GetDirtyMask(void);

/* Transmits one specific page regardless of its dirty bit. */
bool SSD1306_FlushPage(uint8_t page);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_SIMPLE_H */

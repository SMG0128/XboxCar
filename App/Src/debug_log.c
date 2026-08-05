#include "debug_log.h"

#include <stdio.h>
#include <string.h>

#define LOG_RING_MASK (DEBUG_LOG_RING_SIZE - 1U)

/*
 * One transport for one UART. A per-instance handle would suggest the firmware
 * could log to two places, which the PCB cannot do.
 */
static struct {
  uint8_t ring[DEBUG_LOG_RING_SIZE];
  uint16_t head;
  uint16_t tail;
  uint32_t dropped_messages;
  bool enabled;
} g_log = {{0}, 0U, 0U, 0U, true};

static uint16_t Used(void)
{
  return (uint16_t)((g_log.head - g_log.tail) & LOG_RING_MASK);
}

/* One slot is always left empty so head == tail unambiguously means empty. */
static uint16_t Free(void)
{
  return (uint16_t)(LOG_RING_MASK - Used());
}

void DebugLog_Init(void)
{
  g_log.head = 0U;
  g_log.tail = 0U;
  g_log.dropped_messages = 0U;
  g_log.enabled = true;
}

void DebugLog_SetEnabled(bool enabled)
{
  g_log.enabled = enabled;
}

bool DebugLog_IsEnabled(void)
{
  return g_log.enabled;
}

/*
 * All-or-nothing append. Callers rely on never seeing a half line, because a
 * truncated line in a bring-up log reads as corrupted data rather than as a
 * dropped message.
 */
static void Append(const char *text, uint16_t length)
{
  uint16_t index;

  if (length == 0U)
  {
    return;
  }

  if (length > Free())
  {
    ++g_log.dropped_messages;
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    g_log.ring[g_log.head] = (uint8_t)text[index];
    g_log.head = (uint16_t)((g_log.head + 1U) & LOG_RING_MASK);
  }
}

void DebugLog_VPrintf(const char *format, va_list args)
{
  char line[DEBUG_LOG_LINE_MAX];
  int written;

  if (!g_log.enabled || format == NULL)
  {
    return;
  }

  written = vsnprintf(line, sizeof(line), format, args);
  if (written <= 0)
  {
    return;
  }

  /*
   * vsnprintf reports the length it wanted, not the length it wrote. A line
   * longer than the buffer is counted as dropped rather than emitted clipped.
   */
  if ((size_t)written >= sizeof(line))
  {
    ++g_log.dropped_messages;
    return;
  }

  Append(line, (uint16_t)written);
}

void DebugLog_Printf(const char *format, ...)
{
  va_list args;

  if (!g_log.enabled || format == NULL)
  {
    return;
  }

  va_start(args, format);
  DebugLog_VPrintf(format, args);
  va_end(args);
}

void DebugLog_Puts(const char *text)
{
  size_t length;

  if (!g_log.enabled || text == NULL)
  {
    return;
  }

  length = strlen(text);
  if (length > 0xFFFFU)
  {
    ++g_log.dropped_messages;
    return;
  }

  Append(text, (uint16_t)length);
}

bool DebugLog_PopByte(uint8_t *byte)
{
  if (byte == NULL || g_log.tail == g_log.head)
  {
    return false;
  }

  *byte = g_log.ring[g_log.tail];
  g_log.tail = (uint16_t)((g_log.tail + 1U) & LOG_RING_MASK);
  return true;
}

uint16_t DebugLog_Pending(void)
{
  return Used();
}

uint32_t DebugLog_DroppedMessages(void)
{
  return g_log.dropped_messages;
}

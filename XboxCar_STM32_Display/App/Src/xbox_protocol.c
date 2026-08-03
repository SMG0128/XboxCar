#include "xbox_protocol.h"

#include <string.h>

/* Ring index mask. COMM_RX_RING_SIZE is asserted to be a power of two. */
#define RING_MASK (COMM_RX_RING_SIZE - 1U)

/* Commas sit at fixed offsets in a well formed frame. */
static const uint8_t kCommaOffsets[] = {3U, 8U, 14U, 20U, 25U};

uint8_t XboxProtocol_Crc(const uint8_t *data, uint16_t length)
{
  uint8_t crc = 0U;
  uint16_t index;

  if (data == NULL)
  {
    return 0U;
  }

  for (index = 0U; index < length; ++index)
  {
    crc ^= data[index];
  }
  return crc;
}

void XboxProtocol_Init(XboxProtocol *protocol)
{
  if (protocol == NULL)
  {
    return;
  }

  memset(protocol, 0, sizeof(*protocol));
}

void XboxProtocol_PushByte(XboxProtocol *protocol, uint8_t byte)
{
  uint16_t head;
  uint16_t next;

  if (protocol == NULL)
  {
    return;
  }

  head = protocol->head;
  next = (uint16_t)((head + 1U) & RING_MASK);

  /*
   * A full ring means the main loop has fallen behind. Dropping the newest byte
   * corrupts at most the frame in flight, and the parser resynchronises on the
   * next '$'. Overwriting the tail instead would silently corrupt an older
   * frame that is about to be parsed.
   */
  if (next == protocol->tail)
  {
    ++protocol->isr_overflows;
    return;
  }

  protocol->ring[head] = byte;
  protocol->head = next;
  ++protocol->isr_rx_bytes;
}

/* Pops one byte. Returns false when the ring is empty. */
static bool RingPop(XboxProtocol *protocol, uint8_t *byte)
{
  uint16_t tail = protocol->tail;

  if (tail == protocol->head)
  {
    return false;
  }

  *byte = protocol->ring[tail];
  protocol->tail = (uint16_t)((tail + 1U) & RING_MASK);
  return true;
}

static bool ParseDigit(uint8_t value, uint8_t *digit)
{
  if (value < (uint8_t)'0' || value > (uint8_t)'9')
  {
    return false;
  }
  *digit = (uint8_t)(value - (uint8_t)'0');
  return true;
}

static bool ParseUnsigned4(const uint8_t *text, uint16_t *value)
{
  uint16_t result = 0U;
  uint8_t digit;
  uint8_t index;

  for (index = 0U; index < 4U; ++index)
  {
    if (!ParseDigit(text[index], &digit))
    {
      return false;
    }
    result = (uint16_t)(result * 10U + digit);
  }

  *value = result;
  return true;
}

/* Signed five character field: a sign followed by four digits. */
static bool ParseSigned5(const uint8_t *text, int16_t *value)
{
  uint16_t magnitude;

  if (text[0] != (uint8_t)'+' && text[0] != (uint8_t)'-')
  {
    return false;
  }

  if (!ParseUnsigned4(&text[1], &magnitude))
  {
    return false;
  }

  if (magnitude > (uint16_t)MOTOR_SPEED_MAX)
  {
    return false;
  }

  *value = (text[0] == (uint8_t)'-') ? (int16_t)(-(int32_t)magnitude)
                                     : (int16_t)magnitude;
  return true;
}

static bool ParseHexByte(const uint8_t *text, uint8_t *value)
{
  uint8_t result = 0U;
  uint8_t index;
  uint8_t nibble;

  for (index = 0U; index < 2U; ++index)
  {
    if (text[index] >= (uint8_t)'0' && text[index] <= (uint8_t)'9')
    {
      nibble = (uint8_t)(text[index] - (uint8_t)'0');
    }
    else if (text[index] >= (uint8_t)'A' && text[index] <= (uint8_t)'F')
    {
      /* Upper case only, matching the ESP32 "%02X" emitter. */
      nibble = (uint8_t)(text[index] - (uint8_t)'A' + 10U);
    }
    else
    {
      return false;
    }
    result = (uint8_t)((result << 4U) | nibble);
  }

  *value = result;
  return true;
}

/* Four binary digits, most significant first. */
static bool ParseCommand(const uint8_t *text, uint8_t *command)
{
  uint8_t result = 0U;
  uint8_t index;

  for (index = 0U; index < 4U; ++index)
  {
    if (text[index] != (uint8_t)'0' && text[index] != (uint8_t)'1')
    {
      return false;
    }
    result = (uint8_t)((result << 1U) | (text[index] - (uint8_t)'0'));
  }

  *command = result;
  return true;
}

static bool CommandIsKnown(uint8_t command)
{
  switch (command)
  {
    case XBOX_CMD_STOP:
    case XBOX_CMD_FORWARD:
    case XBOX_CMD_REVERSE:
    case XBOX_CMD_TURN_LEFT:
    case XBOX_CMD_TURN_RIGHT:
    case XBOX_CMD_SPIN_LEFT:
    case XBOX_CMD_SPIN_RIGHT:
    case XBOX_CMD_EMERGENCY_STOP:
    case XBOX_CMD_NO_INPUT:
    case XBOX_CMD_DISCONNECTED:
    case XBOX_CMD_ERROR:
      return true;
    default:
      return false;
  }
}

/*
 * Structural check. Separated from value parsing so that a framing fault and a
 * range fault land in different counters.
 */
static bool CheckStructure(const uint8_t *frame)
{
  uint8_t index;

  if (frame[0] != (uint8_t)'$' || frame[1] != (uint8_t)'X' ||
      frame[2] != (uint8_t)'C')
  {
    return false;
  }

  if (frame[XBOX_FRAME_LENGTH - 2U] != (uint8_t)'\r' ||
      frame[XBOX_FRAME_LENGTH - 1U] != (uint8_t)'\n')
  {
    return false;
  }

  for (index = 0U; index < (uint8_t)sizeof(kCommaOffsets); ++index)
  {
    if (frame[kCommaOffsets[index]] != (uint8_t)',')
    {
      return false;
    }
  }

  return true;
}

/*
 * Decides whether a well formed frame's sequence number belongs to the stream
 * we are already following.
 *
 * Returns XBOX_POLL_FRAME to accept, XBOX_POLL_DUPLICATE for an exact repeat,
 * XBOX_POLL_ERROR for anything stale or discontinuous.
 *
 * An ESP32 restart makes the sequence jump backwards to 0000. That is
 * indistinguishable from a stale frame on its own, so the first out-of-window
 * frame is remembered as a candidate and rejected; if the very next frame
 * continues from it, the stream is re-baselined. A restart therefore costs one
 * dropped frame (20 ms) rather than a full communication timeout.
 */
static XboxPollResult ClassifySequence(XboxProtocol *protocol, uint16_t sequence)
{
  uint16_t delta;

  if (!protocol->have_sequence)
  {
    protocol->have_sequence = true;
    protocol->last_sequence = sequence;
    protocol->have_resync_candidate = false;
    protocol->relaxed_sequence = false;
    return XBOX_POLL_FRAME;
  }

  delta = (uint16_t)((sequence + COMM_SEQ_MODULUS - protocol->last_sequence) %
                     COMM_SEQ_MODULUS);

  /*
   * Checked before the relaxed path on purpose. An emitter that has frozen
   * while resending one frame must never look like a recovered link, however
   * long the outage lasted.
   */
  if (delta == 0U)
  {
    ++protocol->stats.duplicate_frames;
    return XBOX_POLL_DUPLICATE;
  }

  /*
   * The link was lost and this is the first new frame since. Whatever the
   * emitter's sequence is now, including 0000 after a reboot, it becomes the
   * new baseline.
   */
  if (protocol->relaxed_sequence)
  {
    protocol->last_sequence = sequence;
    protocol->relaxed_sequence = false;
    protocol->have_resync_candidate = false;
    return XBOX_POLL_FRAME;
  }

  if (delta <= COMM_SEQ_FORWARD_WINDOW)
  {
    /*
     * A gap inside the window means frames were lost in transit. Count it for
     * diagnosis but keep driving: refusing to act would turn packet loss into a
     * stall.
     */
    if (delta > 1U)
    {
      ++protocol->stats.sequence_errors;
    }
    protocol->last_sequence = sequence;
    protocol->have_resync_candidate = false;
    return XBOX_POLL_FRAME;
  }

  /* Out of window: stale frame, or a restarted emitter. */
  if (protocol->have_resync_candidate &&
      sequence == (uint16_t)((protocol->resync_candidate + 1U) % COMM_SEQ_MODULUS))
  {
    protocol->last_sequence = sequence;
    protocol->have_resync_candidate = false;
    return XBOX_POLL_FRAME;
  }

  protocol->have_resync_candidate = true;
  protocol->resync_candidate = sequence;
  ++protocol->stats.sequence_errors;
  return XBOX_POLL_ERROR;
}

/* Validates an assembled 30 byte candidate and, if it passes, fills frame. */
static XboxPollResult ValidateFrame(XboxProtocol *protocol,
                                    XboxControlFrame *frame)
{
  const uint8_t *raw = protocol->frame;
  XboxControlFrame parsed;
  uint8_t received_crc;
  uint8_t calculated_crc;
  uint16_t sequence;
  XboxPollResult sequence_result;

  if (!CheckStructure(raw))
  {
    ++protocol->stats.format_errors;
    return XBOX_POLL_ERROR;
  }

  /* CRC before field parsing: a corrupt frame should not be interpreted. */
  if (!ParseHexByte(&raw[XBOX_FRAME_CRC_OFFSET], &received_crc))
  {
    ++protocol->stats.format_errors;
    return XBOX_POLL_ERROR;
  }

  calculated_crc = XboxProtocol_Crc(&raw[XBOX_FRAME_CRC_FIRST],
                                    XBOX_FRAME_CRC_LAST - XBOX_FRAME_CRC_FIRST + 1U);
  if (received_crc != calculated_crc)
  {
    ++protocol->stats.crc_errors;
    return XBOX_POLL_ERROR;
  }

  if (!ParseCommand(&raw[XBOX_FRAME_CMD_OFFSET], &parsed.command))
  {
    ++protocol->stats.format_errors;
    return XBOX_POLL_ERROR;
  }

  if (!CommandIsKnown(parsed.command))
  {
    ++protocol->stats.range_errors;
    return XBOX_POLL_ERROR;
  }

  if (!ParseSigned5(&raw[XBOX_FRAME_LEFT_OFFSET], &parsed.left) ||
      !ParseSigned5(&raw[XBOX_FRAME_RIGHT_OFFSET], &parsed.right))
  {
    /*
     * Either the field is not a signed decimal or the magnitude exceeds the
     * agreed limit. Both are refusals to drive on untrusted numbers.
     */
    ++protocol->stats.range_errors;
    return XBOX_POLL_ERROR;
  }

  if (!ParseUnsigned4(&raw[XBOX_FRAME_SEQ_OFFSET], &sequence))
  {
    ++protocol->stats.format_errors;
    return XBOX_POLL_ERROR;
  }

  sequence_result = ClassifySequence(protocol, sequence);
  if (sequence_result != XBOX_POLL_FRAME)
  {
    return sequence_result;
  }

  parsed.sequence = sequence;
  parsed.connected = (parsed.command != (uint8_t)XBOX_CMD_DISCONNECTED);
  parsed.emergency = (parsed.command == (uint8_t)XBOX_CMD_EMERGENCY_STOP ||
                      parsed.command == (uint8_t)XBOX_CMD_ERROR);

  /*
   * These commands are defined to carry zeros. Forcing them here means a
   * malformed-but-CRC-valid emitter cannot drive the motors through a safety
   * command.
   */
  parsed.forces_zero = (parsed.command == (uint8_t)XBOX_CMD_STOP ||
                        parsed.command == (uint8_t)XBOX_CMD_EMERGENCY_STOP ||
                        parsed.command == (uint8_t)XBOX_CMD_NO_INPUT ||
                        parsed.command == (uint8_t)XBOX_CMD_DISCONNECTED ||
                        parsed.command == (uint8_t)XBOX_CMD_ERROR);
  if (parsed.forces_zero)
  {
    parsed.left = 0;
    parsed.right = 0;
  }

  ++protocol->stats.valid_frames;

  if (frame != NULL)
  {
    *frame = parsed;
  }
  return XBOX_POLL_FRAME;
}

XboxPollResult XboxProtocol_Poll(XboxProtocol *protocol, XboxControlFrame *frame)
{
  uint8_t byte;

  if (protocol == NULL)
  {
    return XBOX_POLL_IDLE;
  }

  while (RingPop(protocol, &byte))
  {
    /*
     * '$' is the frame delimiter and cannot occur inside a well formed frame,
     * so it always restarts assembly. This is what recovers from noise, a half
     * frame and a truncated frame in a single rule.
     */
    if (byte == (uint8_t)'$')
    {
      bool abandoned = protocol->in_frame;

      protocol->frame[0] = byte;
      protocol->frame_length = 1U;
      protocol->in_frame = true;

      if (abandoned)
      {
        /* A frame was in progress and never completed. */
        ++protocol->stats.format_errors;
        return XBOX_POLL_ERROR;
      }
      continue;
    }

    if (!protocol->in_frame)
    {
      /* Pre-frame noise, for example Bluepad32 boot logging. */
      continue;
    }

    protocol->frame[protocol->frame_length] = byte;
    ++protocol->frame_length;

    if (protocol->frame_length >= XBOX_FRAME_LENGTH)
    {
      XboxPollResult result;

      protocol->in_frame = false;
      protocol->frame_length = 0U;
      result = ValidateFrame(protocol, frame);
      return result;
    }
  }

  return XBOX_POLL_IDLE;
}

void XboxProtocol_ResetSequence(XboxProtocol *protocol)
{
  if (protocol == NULL)
  {
    return;
  }

  /*
   * last_sequence is deliberately retained. Relaxing the baseline is what lets
   * a rebooted emitter back in; keeping the last value is what keeps a frozen
   * one out.
   */
  protocol->relaxed_sequence = true;
  protocol->have_resync_candidate = false;
}

void XboxProtocol_GetStats(const XboxProtocol *protocol, XboxProtocolStats *stats)
{
  if (protocol == NULL || stats == NULL)
  {
    return;
  }

  *stats = protocol->stats;

  /*
   * The byte and overflow counters are owned by the interrupt. A single 32 bit
   * read is atomic on this core, so no critical section is needed.
   */
  stats->rx_bytes = protocol->isr_rx_bytes;
  stats->rx_overflows = protocol->isr_overflows;
}

#include "xbox_protocol.h"

#include <string.h>

/* Ring index mask. COMM_RX_RING_SIZE is asserted to be a power of two. */
#define RING_MASK (COMM_RX_RING_SIZE - 1U)

/* Commas sit at fixed offsets in a well formed frame, one table per version. */
static const uint8_t kCommaOffsetsV1[] = {3U, 8U, 14U, 20U, 25U};
static const uint8_t kCommaOffsetsV2[] = {3U,  8U,  14U, 20U, 26U,
                                          32U, 38U, 44U, 47U, 52U};

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

/*
 * Signed five character field: a sign followed by four digits. limit is the
 * largest accepted magnitude, so a control field and a diagnostic-only raw
 * field can share the parser without sharing a range rule.
 */
static bool ParseSigned5(const uint8_t *text, int16_t *value, int16_t limit)
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

  if (magnitude > (uint16_t)limit)
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
 * Frame length implied by the format byte at index 2, or 0 when the byte does
 * not name a format this build understands.
 */
static uint16_t FrameLengthForFormat(uint8_t format_byte)
{
  if (format_byte == (uint8_t)XBOX_FORMAT_V1_CHAR)
  {
    return XBOX_FRAME_V1_LENGTH;
  }
  if (format_byte == (uint8_t)XBOX_FORMAT_V2_CHAR)
  {
    return XBOX_FRAME_V2_LENGTH;
  }
  return 0U;
}

/*
 * Structural check. Separated from value parsing so that a framing fault and a
 * range fault land in different counters.
 */
static bool CheckStructure(const uint8_t *frame, uint16_t length)
{
  const uint8_t *commas;
  uint8_t comma_count;
  uint8_t index;

  if (frame[0] != (uint8_t)'$' || frame[1] != (uint8_t)'X')
  {
    return false;
  }

  if (frame[length - 2U] != (uint8_t)'\r' || frame[length - 1U] != (uint8_t)'\n')
  {
    return false;
  }

  if (length == XBOX_FRAME_V1_LENGTH)
  {
    commas = kCommaOffsetsV1;
    comma_count = (uint8_t)sizeof(kCommaOffsetsV1);
  }
  else
  {
    commas = kCommaOffsetsV2;
    comma_count = (uint8_t)sizeof(kCommaOffsetsV2);
  }

  for (index = 0U; index < comma_count; ++index)
  {
    if (frame[commas[index]] != (uint8_t)',')
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
  protocol->last_reject_reason = (uint8_t)XBOX_REJECT_SEQUENCE;
  return XBOX_POLL_ERROR;
}

/* Records a rejection reason alongside the counter the caller already bumped. */
static XboxPollResult Reject(XboxProtocol *protocol, XboxRejectReason reason)
{
  protocol->last_reject_reason = (uint8_t)reason;
  return XBOX_POLL_ERROR;
}

/* Validates an assembled candidate and, if it passes, fills frame. */
static XboxPollResult ValidateFrame(XboxProtocol *protocol,
                                    XboxControlFrame *frame)
{
  const uint8_t *raw = protocol->frame;
  const uint16_t length = protocol->expected_length;
  const bool is_v2 = (length == XBOX_FRAME_V2_LENGTH);
  XboxControlFrame parsed;
  uint8_t received_crc;
  uint8_t calculated_crc;
  uint16_t sequence;
  uint16_t crc_first;
  uint16_t crc_last;
  uint16_t crc_offset;
  uint16_t seq_offset;
  XboxPollResult sequence_result;

  memset(&parsed, 0, sizeof(parsed));

  if (!CheckStructure(raw, length))
  {
    ++protocol->stats.format_errors;
    return Reject(protocol, XBOX_REJECT_FORMAT);
  }

  if (is_v2)
  {
    crc_first = XBOX_V2_CRC_FIRST;
    crc_last = XBOX_V2_CRC_LAST;
    crc_offset = XBOX_V2_CRC_OFFSET;
    seq_offset = XBOX_V2_SEQ_OFFSET;
  }
  else
  {
    crc_first = XBOX_V1_CRC_FIRST;
    crc_last = XBOX_V1_CRC_LAST;
    crc_offset = XBOX_V1_CRC_OFFSET;
    seq_offset = XBOX_V1_SEQ_OFFSET;
  }

  /* CRC before field parsing: a corrupt frame should not be interpreted. */
  if (!ParseHexByte(&raw[crc_offset], &received_crc))
  {
    ++protocol->stats.format_errors;
    return Reject(protocol, XBOX_REJECT_FORMAT);
  }

  calculated_crc =
      XboxProtocol_Crc(&raw[crc_first], (uint16_t)(crc_last - crc_first + 1U));
  if (received_crc != calculated_crc)
  {
    ++protocol->stats.crc_errors;
    return Reject(protocol, XBOX_REJECT_CRC);
  }

  if (!ParseCommand(&raw[XBOX_V1_CMD_OFFSET], &parsed.command))
  {
    ++protocol->stats.format_errors;
    return Reject(protocol, XBOX_REJECT_FORMAT);
  }

  if (!CommandIsKnown(parsed.command))
  {
    ++protocol->stats.range_errors;
    return Reject(protocol, XBOX_REJECT_UNKNOWN_COMMAND);
  }

  /*
   * The mixed pair sits at the same offsets in both versions, which is what
   * lets one emitter upgrade without moving the fields the vehicle drives on.
   */
  if (!ParseSigned5(&raw[XBOX_V1_LEFT_OFFSET], &parsed.left,
                    (int16_t)MOTOR_SPEED_MAX) ||
      !ParseSigned5(&raw[XBOX_V1_RIGHT_OFFSET], &parsed.right,
                    (int16_t)MOTOR_SPEED_MAX))
  {
    /*
     * Either the field is not a signed decimal or the magnitude exceeds the
     * agreed limit. Both are refusals to drive on untrusted numbers.
     */
    ++protocol->stats.range_errors;
    return Reject(protocol, XBOX_REJECT_RANGE);
  }

  if (is_v2)
  {
    if (!ParseSigned5(&raw[XBOX_V2_UD_OFFSET], &parsed.up_down,
                      (int16_t)MOTOR_SPEED_MAX) ||
        !ParseSigned5(&raw[XBOX_V2_LR_OFFSET], &parsed.left_right,
                      (int16_t)MOTOR_SPEED_MAX) ||
        !ParseSigned5(&raw[XBOX_V2_RAW_UD_OFFSET], &parsed.raw_up_down,
                      (int16_t)XBOX_RAW_AXIS_MAX) ||
        !ParseSigned5(&raw[XBOX_V2_RAW_LR_OFFSET], &parsed.raw_left_right,
                      (int16_t)XBOX_RAW_AXIS_MAX))
    {
      ++protocol->stats.range_errors;
      return Reject(protocol, XBOX_REJECT_RANGE);
    }

    if (!ParseHexByte(&raw[XBOX_V2_FLAGS_OFFSET], &parsed.flags))
    {
      ++protocol->stats.format_errors;
      return Reject(protocol, XBOX_REJECT_FORMAT);
    }

    parsed.version = 2U;
  }
  else
  {
    /*
     * v1 carries no dimensions of its own. Recover them with the same
     * decomposition the safety limiter uses, so a v1 emitter still produces a
     * sensible display and log instead of two blank axes.
     */
    parsed.up_down = (int16_t)(((int32_t)parsed.left + (int32_t)parsed.right) / 2);
    parsed.left_right =
        (int16_t)(((int32_t)parsed.left - (int32_t)parsed.right) / 2);
    parsed.raw_up_down = 0;
    parsed.raw_left_right = 0;
    parsed.flags = (parsed.command != (uint8_t)XBOX_CMD_DISCONNECTED)
                       ? (uint8_t)(XBOX_FLAG_CONNECTED | XBOX_FLAG_HAS_SAMPLE)
                       : 0U;
    parsed.version = 1U;
  }

  if (!ParseUnsigned4(&raw[seq_offset], &sequence))
  {
    ++protocol->stats.format_errors;
    return Reject(protocol, XBOX_REJECT_FORMAT);
  }

  sequence_result = ClassifySequence(protocol, sequence);
  if (sequence_result != XBOX_POLL_FRAME)
  {
    return sequence_result;
  }

  parsed.sequence = sequence;

  /*
   * Two independent statements of presence must agree. A frame that claims a
   * connected controller while carrying the Disconnected action code is not
   * trusted to drive, so the pessimistic reading wins.
   */
  parsed.connected = ((parsed.flags & XBOX_FLAG_CONNECTED) != 0U) &&
                     (parsed.command != (uint8_t)XBOX_CMD_DISCONNECTED);
  parsed.has_sample = ((parsed.flags & XBOX_FLAG_HAS_SAMPLE) != 0U);
  parsed.emergency = (parsed.command == (uint8_t)XBOX_CMD_EMERGENCY_STOP ||
                      parsed.command == (uint8_t)XBOX_CMD_ERROR ||
                      (parsed.flags & XBOX_FLAG_CONTROL_ERROR) != 0U);

  /*
   * These commands are defined to carry zeros. Forcing them here means a
   * malformed-but-CRC-valid emitter cannot drive the motors through a safety
   * command.
   */
  parsed.forces_zero = (parsed.command == (uint8_t)XBOX_CMD_STOP ||
                        parsed.command == (uint8_t)XBOX_CMD_EMERGENCY_STOP ||
                        parsed.command == (uint8_t)XBOX_CMD_NO_INPUT ||
                        parsed.command == (uint8_t)XBOX_CMD_DISCONNECTED ||
                        parsed.command == (uint8_t)XBOX_CMD_ERROR ||
                        !parsed.connected);
  if (parsed.forces_zero)
  {
    parsed.left = 0;
    parsed.right = 0;
    parsed.up_down = 0;
    parsed.left_right = 0;
  }

  ++protocol->stats.valid_frames;
  if (is_v2)
  {
    ++protocol->stats.v2_frames;
  }
  else
  {
    ++protocol->stats.v1_frames;
  }
  protocol->last_reject_reason = (uint8_t)XBOX_REJECT_NONE;

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
      protocol->expected_length = 0U;
      protocol->in_frame = true;

      if (abandoned)
      {
        /* A frame was in progress and never completed. */
        ++protocol->stats.format_errors;
        return Reject(protocol, XBOX_REJECT_FORMAT);
      }
      continue;
    }

    if (!protocol->in_frame)
    {
      /* Pre-frame noise, for example Bluepad32 boot logging. */
      continue;
    }

    /*
     * Bound the write before it happens. The length decision below guarantees
     * this never trips, but the buffer index is derived from stream data and a
     * guard that costs one comparison is cheaper than trusting that argument.
     */
    if (protocol->frame_length >= XBOX_FRAME_MAX_LENGTH)
    {
      protocol->in_frame = false;
      protocol->frame_length = 0U;
      protocol->expected_length = 0U;
      ++protocol->stats.format_errors;
      return Reject(protocol, XBOX_REJECT_FORMAT);
    }

    protocol->frame[protocol->frame_length] = byte;
    ++protocol->frame_length;

    /*
     * The third byte names the format and therefore the length. Deciding here
     * rather than at completion is what allows two frame sizes to share one
     * streaming assembler: until this point every format looks identical.
     */
    if (protocol->frame_length == 3U)
    {
      protocol->expected_length = FrameLengthForFormat(protocol->frame[2]);
      if (protocol->frame[1] != (uint8_t)'X' || protocol->expected_length == 0U)
      {
        /*
         * Not a frame header this build understands. Abandon assembly and wait
         * for the next '$' rather than consuming bytes into a length we guessed.
         */
        protocol->in_frame = false;
        protocol->frame_length = 0U;
        protocol->expected_length = 0U;
        ++protocol->stats.format_errors;
        return Reject(protocol, XBOX_REJECT_FORMAT);
      }
      continue;
    }

    if (protocol->expected_length != 0U &&
        protocol->frame_length >= protocol->expected_length)
    {
      XboxPollResult result;

      protocol->in_frame = false;
      protocol->frame_length = 0U;
      result = ValidateFrame(protocol, frame);
      protocol->expected_length = 0U;
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

uint8_t XboxProtocol_GetLastRejectReason(const XboxProtocol *protocol)
{
  if (protocol == NULL)
  {
    return (uint8_t)XBOX_REJECT_NONE;
  }
  return protocol->last_reject_reason;
}

const char *XboxProtocol_RejectReasonName(uint8_t reason)
{
  switch (reason)
  {
    case XBOX_REJECT_FORMAT:
      return "FORMAT_ERROR";
    case XBOX_REJECT_CRC:
      return "CRC_ERROR";
    case XBOX_REJECT_RANGE:
      return "RANGE_ERROR";
    case XBOX_REJECT_SEQUENCE:
      return "SEQUENCE_ERROR";
    case XBOX_REJECT_UNKNOWN_COMMAND:
      return "UNKNOWN_COMMAND";
    case XBOX_REJECT_NONE:
    default:
      return "NONE";
  }
}

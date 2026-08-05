#ifndef XBOX_PROTOCOL_H
#define XBOX_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * XboxCar ESP32 -> STM32 control frame receiver.
 *
 * Two frame formats are accepted. Both start with '$', end with CRLF, carry an
 * XOR checksum over the bytes from 'X' up to and including the last SEQ digit,
 * and are distinguished by the third byte.
 *
 * v1, 30 bytes, legacy:
 *
 *     $XC,<CMD>,<LEFT>,<RIGHT>,<SEQ>,<CRC>\r\n
 *     $XC,0001,+0800,+0800,0025,1D\r\n
 *
 * v2, 57 bytes, current:
 *
 *     $XD,<CMD>,<LEFT>,<RIGHT>,<UD>,<LR>,<RAW_UD>,<RAW_LR>,<FLAGS>,<SEQ>,<CRC>\r\n
 *     $XD,0001,+0800,+0800,+0800,+0000,-0511,+0000,03,0025,4F\r\n
 *
 * v2 exists because v1 only carries the already-mixed LEFT/RIGHT wheel pair.
 * The OLED and the diagnostics have to show the two control dimensions the
 * operator actually moves - forward/back and left/right - and those cannot be
 * recovered from a mixed pair once the steering gain and the saturation scaler
 * have been applied. v2 therefore transmits them explicitly, along with the raw
 * stick counts and an explicit controller-connected flag.
 *
 * v1 remains accepted so an ESP32 running older firmware still drives the
 * vehicle. For a v1 frame the extra fields are derived: UD/LR from the mixed
 * pair, and the connected flag from the command code.
 *
 * Design constraints this module exists to satisfy:
 *   - The interrupt handler must not parse. It only pushes bytes into a ring.
 *   - Parsing runs on the main loop and tolerates noise, split frames, glued
 *     frames, half frames and ring overflow, resynchronising on the next '$'.
 *   - Only a frame that passes every check may refresh the communication
 *     watchdog. Malformed, unknown, duplicate and stale frames must not.
 *
 * The module contains no STM32 HAL dependency and is compiled unchanged by the
 * host test build.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"

/* Frame lengths, both fixed once the format byte has been seen. */
#define XBOX_FRAME_V1_LENGTH 30U
#define XBOX_FRAME_V2_LENGTH 57U
#define XBOX_FRAME_MAX_LENGTH XBOX_FRAME_V2_LENGTH

/* Retained under the old name: several callers still mean "a v1 frame". */
#define XBOX_FRAME_LENGTH XBOX_FRAME_V1_LENGTH

/* Format byte at index 2, after the '$' and the 'X'. */
#define XBOX_FORMAT_V1_CHAR 'C'
#define XBOX_FORMAT_V2_CHAR 'D'

/* Field offsets inside a complete v1 frame. */
#define XBOX_V1_CMD_OFFSET 4U
#define XBOX_V1_LEFT_OFFSET 9U
#define XBOX_V1_RIGHT_OFFSET 15U
#define XBOX_V1_SEQ_OFFSET 21U
#define XBOX_V1_CRC_OFFSET 26U
#define XBOX_V1_CRC_FIRST 1U
#define XBOX_V1_CRC_LAST 24U

/* Backwards compatible spellings of the v1 offsets. */
#define XBOX_FRAME_CMD_OFFSET XBOX_V1_CMD_OFFSET
#define XBOX_FRAME_LEFT_OFFSET XBOX_V1_LEFT_OFFSET
#define XBOX_FRAME_RIGHT_OFFSET XBOX_V1_RIGHT_OFFSET
#define XBOX_FRAME_SEQ_OFFSET XBOX_V1_SEQ_OFFSET
#define XBOX_FRAME_CRC_OFFSET XBOX_V1_CRC_OFFSET
#define XBOX_FRAME_CRC_FIRST XBOX_V1_CRC_FIRST
#define XBOX_FRAME_CRC_LAST XBOX_V1_CRC_LAST

/* Field offsets inside a complete v2 frame. */
#define XBOX_V2_CMD_OFFSET 4U
#define XBOX_V2_LEFT_OFFSET 9U
#define XBOX_V2_RIGHT_OFFSET 15U
#define XBOX_V2_UD_OFFSET 21U
#define XBOX_V2_LR_OFFSET 27U
#define XBOX_V2_RAW_UD_OFFSET 33U
#define XBOX_V2_RAW_LR_OFFSET 39U
#define XBOX_V2_FLAGS_OFFSET 45U
#define XBOX_V2_SEQ_OFFSET 48U
#define XBOX_V2_CRC_OFFSET 53U
#define XBOX_V2_CRC_FIRST 1U
#define XBOX_V2_CRC_LAST 51U

/*
 * Raw stick counts are diagnostic only and never reach the motors, so they are
 * accepted across the whole width the five character field can express rather
 * than being pinned to one controller's nominal range.
 */
#define XBOX_RAW_AXIS_MAX 9999

/* v2 FLAGS bits, transmitted as two upper case hex digits. */
#define XBOX_FLAG_CONNECTED 0x01U
#define XBOX_FLAG_HAS_SAMPLE 0x02U
#define XBOX_FLAG_EMERGENCY 0x04U
#define XBOX_FLAG_CONTROL_ERROR 0x08U

/* Action codes, mirroring the ESP32 DriveCommand enum. */
typedef enum {
  XBOX_CMD_STOP = 0x0,
  XBOX_CMD_FORWARD = 0x1,
  XBOX_CMD_REVERSE = 0x2,
  XBOX_CMD_TURN_LEFT = 0x3,
  XBOX_CMD_TURN_RIGHT = 0x4,
  XBOX_CMD_SPIN_LEFT = 0x5,
  XBOX_CMD_SPIN_RIGHT = 0x6,
  XBOX_CMD_EMERGENCY_STOP = 0x7,
  XBOX_CMD_NO_INPUT = 0x8,
  XBOX_CMD_DISCONNECTED = 0x9,
  XBOX_CMD_ERROR = 0xF
} XboxCommand;

/* Why the most recent frame was rejected. Mirrors the counters. */
typedef enum {
  XBOX_REJECT_NONE = 0,
  XBOX_REJECT_FORMAT = 1,
  XBOX_REJECT_CRC = 2,
  XBOX_REJECT_RANGE = 3,
  XBOX_REJECT_SEQUENCE = 4,
  XBOX_REJECT_UNKNOWN_COMMAND = 5
} XboxRejectReason;

/* One accepted control frame. */
typedef struct {
  int16_t left;
  int16_t right;

  /*
   * The two operator dimensions, -MOTOR_SPEED_MAX..+MOTOR_SPEED_MAX.
   * up_down > 0 drives forward, left_right > 0 steers right.
   * Transmitted directly by v2, derived from the mixed pair for v1.
   */
  int16_t up_down;
  int16_t left_right;

  /* Raw stick counts, already normalised to the vehicle sign convention. */
  int16_t raw_up_down;
  int16_t raw_left_right;

  uint16_t sequence;
  uint8_t command;
  uint8_t flags;   /* v2 FLAGS byte; synthesised for v1 */
  uint8_t version; /* 1 or 2 */

  /* False when the controller is not present. */
  bool connected;
  /* True when the emitter has seen at least one stick sample. */
  bool has_sample;
  /* True for EmergencyStop / Error. Latches the vehicle until a recovery run. */
  bool emergency;
  /* True when the command itself forces both wheels to zero. */
  bool forces_zero;
} XboxControlFrame;

/* Outcome of a single XboxProtocol_Poll call. */
typedef enum {
  /* Ring drained, nothing further to report. */
  XBOX_POLL_IDLE = 0,
  /* A frame passed every check. Refreshes the watchdog. */
  XBOX_POLL_FRAME = 1,
  /* Well formed but the sequence number repeats the previous frame. */
  XBOX_POLL_DUPLICATE = 2,
  /* Rejected: framing, CRC, range, unknown command or stale sequence. */
  XBOX_POLL_ERROR = 3
} XboxPollResult;

typedef struct {
  uint32_t rx_bytes;
  uint32_t valid_frames;
  uint32_t crc_errors;
  uint32_t format_errors;
  uint32_t range_errors;
  uint32_t sequence_errors;
  uint32_t duplicate_frames;
  uint32_t rx_overflows;
  uint32_t v1_frames;
  uint32_t v2_frames;
} XboxProtocolStats;

typedef struct {
  /* Single producer (interrupt) / single consumer (main loop) byte ring. */
  uint8_t ring[COMM_RX_RING_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint32_t isr_rx_bytes;
  volatile uint32_t isr_overflows;

  /* Frame assembly, main loop only. */
  uint8_t frame[XBOX_FRAME_MAX_LENGTH];
  uint16_t frame_length;
  /* Zero until the format byte has been seen, then 30 or 57. */
  uint16_t expected_length;
  bool in_frame;

  /* Sequence tracking. */
  bool have_sequence;
  uint16_t last_sequence;
  bool have_resync_candidate;
  uint16_t resync_candidate;
  /*
   * Set after a link loss. Accepts the next frame whatever its sequence, so a
   * restarted emitter reconnects at once, but an exact repeat of the last
   * sequence is still a duplicate. Without that exception a stuck emitter
   * resending one frame would bounce between timeout and online forever.
   */
  bool relaxed_sequence;

  /* Why the most recent rejection happened, for the diagnostic log. */
  uint8_t last_reject_reason;

  XboxProtocolStats stats;
} XboxProtocol;

/* Resets every buffer, counter and sequence baseline. */
void XboxProtocol_Init(XboxProtocol *protocol);

/*
 * Pushes one received byte. Safe to call from an interrupt; performs no
 * parsing. A full ring drops the byte and counts an overflow.
 */
void XboxProtocol_PushByte(XboxProtocol *protocol, uint8_t byte);

/*
 * Consumes buffered bytes until one outcome is produced or the ring runs dry.
 * Call repeatedly until XBOX_POLL_IDLE is returned.
 *
 * frame is written only when the result is XBOX_POLL_FRAME.
 */
XboxPollResult XboxProtocol_Poll(XboxProtocol *protocol, XboxControlFrame *frame);

/*
 * Relaxes the sequence baseline so the next well formed frame is accepted
 * whatever its sequence number. Call when the link is considered lost.
 *
 * An exact repeat of the last accepted sequence is still rejected as a
 * duplicate, so a frozen emitter cannot masquerade as a recovered link.
 */
void XboxProtocol_ResetSequence(XboxProtocol *protocol);

/* Snapshot of the counters. Safe against a concurrent interrupt push. */
void XboxProtocol_GetStats(const XboxProtocol *protocol, XboxProtocolStats *stats);

/* Reason code for the most recent rejection. */
uint8_t XboxProtocol_GetLastRejectReason(const XboxProtocol *protocol);

/* Stable short name for a reject reason, for the diagnostic log. */
const char *XboxProtocol_RejectReasonName(uint8_t reason);

/* XOR checksum helper, exposed for tests and for frame construction. */
uint8_t XboxProtocol_Crc(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* XBOX_PROTOCOL_H */

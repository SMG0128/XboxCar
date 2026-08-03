#ifndef XBOX_PROTOCOL_H
#define XBOX_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * XboxCar ESP32 -> STM32 control frame receiver.
 *
 * Fixed 30 byte ASCII frame:
 *
 *     $XC,<CMD>,<LEFT>,<RIGHT>,<SEQ>,<CRC>\r\n
 *     $XC,0001,+0800,+0800,0025,1D\r\n
 *
 * CRC is an 8 bit XOR over the payload starting at 'X' (index 1) and ending at
 * the last SEQ digit (index 24) inclusive. The '$', the CRC field itself and
 * the CRLF are excluded. It is emitted as two upper case hex digits.
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

/* A complete frame is always exactly this long. */
#define XBOX_FRAME_LENGTH 30U

/* Field offsets inside a complete frame. */
#define XBOX_FRAME_CMD_OFFSET 4U
#define XBOX_FRAME_LEFT_OFFSET 9U
#define XBOX_FRAME_RIGHT_OFFSET 15U
#define XBOX_FRAME_SEQ_OFFSET 21U
#define XBOX_FRAME_CRC_OFFSET 26U

/* CRC coverage, inclusive on both ends. */
#define XBOX_FRAME_CRC_FIRST 1U
#define XBOX_FRAME_CRC_LAST 24U

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

/* One accepted control frame. */
typedef struct {
  int16_t left;
  int16_t right;
  uint16_t sequence;
  uint8_t command;
  /* False for Disconnected. The controller is not present. */
  bool connected;
  /* True for EmergencyStop. Latches the vehicle until a recovery run. */
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
} XboxProtocolStats;

typedef struct {
  /* Single producer (interrupt) / single consumer (main loop) byte ring. */
  uint8_t ring[COMM_RX_RING_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint32_t isr_rx_bytes;
  volatile uint32_t isr_overflows;

  /* Frame assembly, main loop only. */
  uint8_t frame[XBOX_FRAME_LENGTH];
  uint16_t frame_length;
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

/* XOR checksum helper, exposed for tests and for frame construction. */
uint8_t XboxProtocol_Crc(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* XBOX_PROTOCOL_H */

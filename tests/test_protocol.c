/*
 * Protocol receiver tests.
 *
 * These drive the real firmware parser through the real control system entry
 * point. Nothing here reimplements framing or checksums; the only algorithm the
 * test owns is the frame builder, which exists so that a "corrupt" frame is
 * corrupted on purpose rather than by accident.
 */

#include <string.h>

#include "test_framework.h"
#include "test_helpers.h"

/* Drains the ring and returns the first non-idle outcome. */
static XboxPollResult PollOnce(XboxProtocol *protocol, XboxControlFrame *frame)
{
  return XboxProtocol_Poll(protocol, frame);
}

static void PushRaw(XboxProtocol *protocol, const char *text)
{
  size_t index;
  for (index = 0U; index < strlen(text); ++index)
  {
    XboxProtocol_PushByte(protocol, (uint8_t)text[index]);
  }
}

static void PushRawLen(XboxProtocol *protocol, const char *data, size_t length)
{
  size_t index;
  for (index = 0U; index < length; ++index)
  {
    XboxProtocol_PushByte(protocol, (uint8_t)data[index]);
  }
}

static void TestCrcMatchesSpecification(void)
{
  TEST_CASE("crc matches the documented example");

  /*
   * The protocol document pins this down: XOR over "XC,0001,+0800,+0800,0025"
   * is 0x1D. If this drifts, the ESP32 and STM32 have silently diverged.
   */
  const char payload[] = "XC,0001,+0800,+0800,0025";
  uint8_t crc = XboxProtocol_Crc((const uint8_t *)payload,
                                 (uint16_t)(sizeof(payload) - 1U));
  TEST_EQ(crc, 0x1D);
}

static void TestBuilderProducesDocumentedFrame(void)
{
  char frame[XBOX_FRAME_LENGTH + 1U];
  size_t length;

  TEST_CASE("frame builder reproduces the documented frame");

  length = TestHelper_BuildFrame(frame, 0x1, 800, 800, 25U);
  TEST_EQ(length, XBOX_FRAME_LENGTH);
  TEST_CHECK(strcmp(frame, "$XC,0001,+0800,+0800,0025,1D\r\n") == 0);
}

static void TestNormalFrameAccepted(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("well formed frame is accepted");

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 800, -350, 25U);
  PushRaw(&protocol, text);

  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 800);
  TEST_EQ(frame.right, -350);
  TEST_EQ(frame.sequence, 25);
  TEST_EQ(frame.command, 0x1);
  TEST_CHECK(frame.connected);
  TEST_CHECK(!frame.emergency);
  TEST_EQ(protocol.stats.valid_frames, 1);
}

static void TestCrcErrorRejected(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("crc error is rejected and counted");

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 800, 800, 25U);
  /* Flip the checksum only; every other byte stays valid. */
  text[XBOX_FRAME_CRC_OFFSET] = (text[XBOX_FRAME_CRC_OFFSET] == 'A') ? 'B' : 'A';
  PushRaw(&protocol, text);

  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
  TEST_EQ(protocol.stats.crc_errors, 1);
  TEST_EQ(protocol.stats.valid_frames, 0);
}

static void TestFormatErrors(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;

  TEST_CASE("structural faults are rejected");

  /* Wrong header. */
  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, "$XZ,0001,+0800,+0800,0025,1D\r\n");
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
  TEST_EQ(protocol.stats.format_errors, 1);

  /* Missing CRLF: 30 bytes but the tail is wrong. */
  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, "$XC,0001,+0800,+0800,0025,1D!!");
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
  TEST_EQ(protocol.stats.format_errors, 1);

  /* Comma replaced by a digit. */
  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, "$XC.0001,+0800,+0800,0025,1D\r\n");
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
  TEST_EQ(protocol.stats.format_errors, 1);
}

static void TestEmptyAndNonNumericFields(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;

  TEST_CASE("empty and non-numeric fields are rejected");

  /*
   * An empty LEFT field shortens the frame, so the 30 byte window swallows the
   * CRLF and the structure check catches it. Either way the frame must not be
   * acted on.
   */
  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, "$XC,0001,,+0800,0025,1D\r\n$XC,");
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);

  /* Letters where digits belong. */
  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, "$XC,0001,+08O0,+0800,0025,1D\r\n");
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
  TEST_CHECK(protocol.stats.valid_frames == 0U);

  /* CMD digits must be binary. */
  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, "$XC,0021,+0800,+0800,0025,1D\r\n");
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
  TEST_EQ(protocol.stats.valid_frames, 0);

  /* Lower case hex is not accepted: the emitter uses %02X. */
  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, "$XC,0001,+0800,+0800,0025,1d\r\n");
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
  TEST_EQ(protocol.stats.valid_frames, 0);
}

static void TestSpeedRangeRejected(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("out of range speed is rejected");

  /* 1001 exceeds the agreed maximum by one. */
  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 1001, 800, 25U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
  TEST_EQ(protocol.stats.range_errors, 1);

  /* The boundary itself is legal. */
  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 1000, -1000, 26U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 1000);
  TEST_EQ(frame.right, -1000);

  /* A missing sign is a malformed signed field. */
  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, "$XC,0001,00800,+0800,0025,1D\r\n");
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
}

static void TestUnknownCommandRejected(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("unknown action code is rejected");

  /* 0b1010 is not in the documented set. */
  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0xA, 0, 0, 25U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
  TEST_EQ(protocol.stats.range_errors, 1);

  /* 0b1111 is defined, and forces both wheels to zero. */
  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0xF, 0, 0, 26U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_CHECK(frame.forces_zero);
}

static void TestSafetyCommandsForceZero(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("safety commands force both wheels to zero");

  /*
   * A checksum-valid emergency frame carrying non-zero speeds must still not
   * be able to drive the motors.
   */
  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x7, 900, 900, 5U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 0);
  TEST_EQ(frame.right, 0);
  TEST_CHECK(frame.emergency);

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x9, 900, 900, 6U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 0);
  TEST_CHECK(!frame.connected);

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x0, 900, -900, 7U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 0);
  TEST_EQ(frame.right, 0);

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0xF, 900, 900, 8U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 0);
  TEST_EQ(frame.right, 0);
  TEST_CHECK(frame.emergency);
}

static void TestLeadingNoiseIgnored(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("bluepad32 style log noise before a frame is ignored");

  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, "BTstack up and running at 00:1A:7D:DA:71:13\r\n");
  PushRaw(&protocol, "Device connected, index=0\n");
  TestHelper_BuildFrame(text, 0x1, 500, 500, 7U);
  PushRaw(&protocol, text);

  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 500);
}

static void TestSplitFrameReassembled(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("frame split across reads is reassembled");

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 640, 640, 8U);

  /* First half only: nothing should be produced yet. */
  PushRawLen(&protocol, text, 13U);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_IDLE);

  /* Remainder arrives later. */
  PushRawLen(&protocol, text + 13, XBOX_FRAME_LENGTH - 13U);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 640);
}

static void TestGluedFramesBothParsed(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char first[XBOX_FRAME_LENGTH + 1U];
  char second[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("two frames in one burst are both parsed");

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(first, 0x1, 100, 100, 10U);
  TestHelper_BuildFrame(second, 0x1, 200, 200, 11U);
  PushRaw(&protocol, first);
  PushRaw(&protocol, second);

  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 100);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 200);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_IDLE);
}

static void TestHalfFrameThenResync(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("truncated frame resynchronises on the next header");

  XboxProtocol_Init(&protocol);
  /* A frame that stops halfway, then a complete one. */
  PushRaw(&protocol, "$XC,0001,+0800,+08");
  TestHelper_BuildFrame(text, 0x1, 300, 300, 12U);
  PushRaw(&protocol, text);

  /* The abandoned frame is reported as an error. */
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
  /* The following frame still parses cleanly. */
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 300);
}

static void TestOverlongGarbageRecovers(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];
  int index;

  TEST_CASE("overlong garbage after a header still recovers");

  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, "$");
  for (index = 0; index < 120; ++index)
  {
    XboxProtocol_PushByte(&protocol, (uint8_t)'Z');
  }
  TestHelper_BuildFrame(text, 0x1, 450, 450, 13U);
  PushRaw(&protocol, text);

  /* Drain until the good frame appears; the junk must not be accepted. */
  {
    XboxPollResult result;
    int guard = 0;
    bool found = false;

    do
    {
      result = PollOnce(&protocol, &frame);
      if (result == XBOX_POLL_FRAME)
      {
        found = true;
        break;
      }
      ++guard;
    } while (result != XBOX_POLL_IDLE && guard < 20);

    TEST_CHECK(found);
    TEST_EQ(frame.left, 450);
  }
}

static void TestRingOverflowCounted(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  uint32_t index;

  TEST_CASE("ring overflow is counted and recovery still works");

  XboxProtocol_Init(&protocol);

  /* Flood well past the ring capacity without ever draining. */
  for (index = 0U; index < COMM_RX_RING_SIZE * 2U; ++index)
  {
    XboxProtocol_PushByte(&protocol, (uint8_t)'Z');
  }
  TEST_CHECK(protocol.isr_overflows > 0U);

  /* Drain the junk. */
  while (PollOnce(&protocol, &frame) != XBOX_POLL_IDLE)
  {
  }

  /* A clean frame after the flood is still accepted. */
  {
    char text[XBOX_FRAME_LENGTH + 1U];
    TestHelper_BuildFrame(text, 0x1, 700, 700, 14U);
    PushRaw(&protocol, text);
    TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
    TEST_EQ(frame.left, 700);
  }
}

static void TestDuplicateSequence(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("repeated sequence number is reported as a duplicate");

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 500, 500, 40U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);

  /* Byte-identical repeat. */
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_DUPLICATE);
  TEST_EQ(protocol.stats.duplicate_frames, 1);
  /* A duplicate is not a valid frame for watchdog purposes. */
  TEST_EQ(protocol.stats.valid_frames, 1);
}

static void TestSequenceGapAccepted(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("a forward gap is accepted but counted");

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 500, 500, 40U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);

  /* Five frames lost in transit; driving must continue. */
  TestHelper_BuildFrame(text, 0x1, 520, 520, 46U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 520);
  TEST_EQ(protocol.stats.sequence_errors, 1);
}

static void TestStaleSequenceRejected(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("a stale frame is rejected");

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 500, 500, 4000U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);

  /* Far behind the baseline: a delayed retransmission, not new information. */
  TestHelper_BuildFrame(text, 0x1, 900, 900, 1000U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);
}

static void TestSequenceWrap(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("sequence wrap from 9999 to 0000 is continuous");

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 500, 500, 9999U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);

  TestHelper_BuildFrame(text, 0x1, 510, 510, 0U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 510);
  /* Wrapping is normal, not an anomaly. */
  TEST_EQ(protocol.stats.sequence_errors, 0);
}

static void TestEsp32RestartRebaselines(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("esp32 restart re-baselines after one dropped frame");

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 500, 500, 5000U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);

  /*
   * The emitter reboots and restarts at 0000. That is indistinguishable from a
   * stale frame on its own, so the first one is refused.
   */
  TestHelper_BuildFrame(text, 0x0, 0, 0, 0U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_ERROR);

  /* The next frame continues the new stream, which proves the restart. */
  TestHelper_BuildFrame(text, 0x1, 120, 120, 1U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 120);

  /* And the stream then runs normally. */
  TestHelper_BuildFrame(text, 0x1, 130, 130, 2U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 130);
}

static void TestSequenceResetAcceptsAnything(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  char text[XBOX_FRAME_LENGTH + 1U];

  TEST_CASE("dropping the baseline accepts the next frame immediately");

  XboxProtocol_Init(&protocol);
  TestHelper_BuildFrame(text, 0x1, 500, 500, 8000U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);

  /* This is what the control system does on a communication timeout. */
  XboxProtocol_ResetSequence(&protocol);

  TestHelper_BuildFrame(text, 0x1, 300, 300, 3U);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.left, 300);
}

static void TestV2FrameAcceptedWithOperatorAxes(void)
{
  XboxProtocol protocol;
  XboxControlFrame frame;
  const char text[] =
      "$XD,0100,+0800,+0350,+0800,+0529,+0511,+0300,03,0000,3C\r\n";

  TEST_CASE("v2 frame preserves axes, raw sticks and flags");

  XboxProtocol_Init(&protocol);
  PushRaw(&protocol, text);
  TEST_EQ(PollOnce(&protocol, &frame), XBOX_POLL_FRAME);
  TEST_EQ(frame.version, 2);
  TEST_EQ(frame.left, 800);
  TEST_EQ(frame.right, 350);
  TEST_EQ(frame.up_down, 800);
  TEST_EQ(frame.left_right, 529);
  TEST_EQ(frame.raw_up_down, 511);
  TEST_EQ(frame.raw_left_right, 300);
  TEST_EQ(frame.flags, XBOX_FLAG_CONNECTED | XBOX_FLAG_HAS_SAMPLE);
  TEST_CHECK(frame.connected);
  TEST_CHECK(frame.has_sample);
}

int run_protocol_tests(void)
{
  printf("-- protocol --\n");
  TestCrcMatchesSpecification();
  TestBuilderProducesDocumentedFrame();
  TestNormalFrameAccepted();
  TestCrcErrorRejected();
  TestFormatErrors();
  TestEmptyAndNonNumericFields();
  TestSpeedRangeRejected();
  TestUnknownCommandRejected();
  TestSafetyCommandsForceZero();
  TestLeadingNoiseIgnored();
  TestSplitFrameReassembled();
  TestGluedFramesBothParsed();
  TestHalfFrameThenResync();
  TestOverlongGarbageRecovers();
  TestRingOverflowCounted();
  TestDuplicateSequence();
  TestSequenceGapAccepted();
  TestStaleSequenceRejected();
  TestSequenceWrap();
  TestEsp32RestartRebaselines();
  TestSequenceResetAcceptsAnything();
  TestV2FrameAcceptedWithOperatorAxes();
  return 0;
}

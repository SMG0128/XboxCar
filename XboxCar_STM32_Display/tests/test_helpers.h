#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

/*
 * Shared helpers for the host tests.
 *
 * The frame builder computes a real XOR checksum rather than carrying a table
 * of hand-written frames, so a test that wants a corrupt frame has to corrupt
 * it explicitly. That keeps "valid" and "invalid" honestly distinct.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "control_system.h"
#include "xbox_protocol.h"

/*
 * Builds "$XC,<cmd>,<left>,<right>,<seq>,<crc>\r\n" into out, which must hold
 * at least XBOX_FRAME_LENGTH + 1 bytes. Returns the length written.
 */
size_t TestHelper_BuildFrame(char *out, uint8_t command, int16_t left,
                             int16_t right, uint16_t sequence);

/* Pushes a NUL-terminated string into the control system's receive ring. */
void TestHelper_PushString(ControlSystem *system, const char *text);

/* Pushes an explicit byte range, allowing embedded NULs and partial frames. */
void TestHelper_PushBytes(ControlSystem *system, const char *data, size_t length);

/*
 * Builds a frame and pushes it. Returns the sequence number used so the caller
 * can chain increments without tracking them separately.
 */
uint16_t TestHelper_PushFrame(ControlSystem *system, uint8_t command,
                              int16_t left, int16_t right, uint16_t sequence);

/*
 * Advances virtual time by stepping the control system one period at a time.
 * Returns the new timestamp. No real waiting is involved anywhere in the tests.
 */
uint32_t TestHelper_RunFor(ControlSystem *system, uint32_t now_ms,
                           uint32_t duration_ms, const SensorSnapshot *sensors);

/* Populates a snapshot where every sensor is valid and reports the same distance. */
void TestHelper_UniformSensors(SensorSnapshot *sensors, uint16_t distance_mm);

/* Clears a snapshot to "no sensors available". */
void TestHelper_NoSensors(SensorSnapshot *sensors);

#endif /* TEST_HELPERS_H */

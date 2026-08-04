#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Up to four HC-SR04 ultrasonic rangefinders, with the configured channels
 * polled one at a time. ULTRASONIC_ENABLED_MASK excludes unsafe or absent
 * channels from both GPIO access and the round robin.
 *
 * The module is a non-blocking state machine driven by repeated Update calls.
 * It never spins, never delays and never touches a peripheral directly: the
 * microsecond clock and the two GPIO operations arrive through a small callback
 * table, which is what makes the whole thing testable on the host with virtual
 * time.
 *
 * Only one sensor is ever active. Firing them together would let one burst be
 * heard by another sensor's receiver and produce a confident wrong reading.
 *
 * The PB0/PB1 channel is disabled because PB1 is not 5 V tolerant. Enabled
 * ECHO pins must be verified as FT for the fitted MCU and used without internal
 * pull-up or pull-down resistors.
 */

#include <stdbool.h>
#include <stdint.h>

#include "app_config.h"
#include "app_types.h"

typedef enum {
  ULTRASONIC_STATE_IDLE = 0,
  ULTRASONIC_STATE_TRIGGER_HIGH = 1,
  ULTRASONIC_STATE_WAIT_ECHO_RISE = 2,
  ULTRASONIC_STATE_WAIT_ECHO_FALL = 3,
  ULTRASONIC_STATE_COMPLETE = 4,
  ULTRASONIC_STATE_TIMEOUT = 5,
  ULTRASONIC_STATE_COOLDOWN = 6
} UltrasonicState;

/* Platform operations the module needs. All must be non-blocking. */
typedef struct {
  /* Free running microsecond counter. Wrap-around is handled by the module. */
  uint32_t (*get_micros)(void);
  /* Drives one sensor's TRIG line. */
  void (*set_trigger)(uint8_t sensor, bool level);
  /* Reads one sensor's ECHO line. */
  bool (*read_echo)(uint8_t sensor);
} UltrasonicHal;

typedef struct {
  /* Most recent accepted raw measurement. */
  uint16_t raw_mm;
  /* Median of the recent accepted measurements. */
  uint16_t filtered_mm;
  /* Last accepted measurement, retained across failures. */
  uint16_t last_valid_mm;

  /* Rolling window of accepted samples, oldest overwritten first. */
  uint16_t samples[ULTRASONIC_MEDIAN_MAX];
  uint8_t sample_count;
  uint8_t sample_index;

  uint32_t consecutive_failures;
  uint32_t consecutive_successes;
  uint32_t timeout_count;
  uint32_t reject_count;

  /* A reading has been accepted and no fault is latched. */
  bool valid;
  bool faulted;
} UltrasonicSensor;

typedef struct {
  UltrasonicHal hal;
  UltrasonicSensor sensor[ULTRASONIC_COUNT];

  UltrasonicState state;
  uint8_t active;          /* sensor currently being measured */
  uint32_t state_entry_us; /* when the current state was entered */
  uint32_t echo_rise_us;   /* timestamp of the observed rising edge */
  bool enabled;
} Ultrasonic;

/*
 * Prepares the module and stores the callback table. Every sensor starts
 * invalid: no distance is reported until a measurement has actually been
 * accepted. Passing a table with any null member leaves the module disabled.
 */
void Ultrasonic_Init(Ultrasonic *ultrasonic, const UltrasonicHal *hal);

/* Enables or disables polling. A disabled module reports no valid sensors. */
void Ultrasonic_SetEnabled(Ultrasonic *ultrasonic, bool enabled);

/* True when polling is active. */
bool Ultrasonic_IsEnabled(const Ultrasonic *ultrasonic);

/*
 * Advances the state machine. Safe to call as often as the main loop likes;
 * it does exactly as much work as the elapsed time allows.
 */
void Ultrasonic_Update(Ultrasonic *ultrasonic);

/* Filtered distance for one sensor, or 0 when it has no valid reading. */
uint16_t Ultrasonic_GetDistance(const Ultrasonic *ultrasonic, uint8_t sensor);

/* Most recent accepted raw distance, or 0. */
uint16_t Ultrasonic_GetRawDistance(const Ultrasonic *ultrasonic, uint8_t sensor);

/* Bit per sensor: a usable, recent reading is available. */
uint8_t Ultrasonic_GetValidMask(const Ultrasonic *ultrasonic);

/* Bit per sensor: too many consecutive failures, the sensor is distrusted. */
uint8_t Ultrasonic_GetFaultMask(const Ultrasonic *ultrasonic);

/* Cumulative echo timeouts for one sensor. */
uint32_t Ultrasonic_GetTimeoutCount(const Ultrasonic *ultrasonic, uint8_t sensor);

/* Current state, exposed for diagnostics and tests. */
UltrasonicState Ultrasonic_GetState(const Ultrasonic *ultrasonic);

/* Sensor currently being measured. */
uint8_t Ultrasonic_GetActiveSensor(const Ultrasonic *ultrasonic);

/*
 * Converts an echo pulse width to millimetres. Exposed for tests.
 * Returns 0 when the pulse is outside the accepted band.
 */
uint16_t Ultrasonic_PulseToMillimetres(uint32_t pulse_us);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H */

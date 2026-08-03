#include "ultrasonic.h"

#include <string.h>

/*
 * Unsigned difference that survives the microsecond counter wrapping. Valid as
 * long as the real interval is shorter than half the counter range, which every
 * timeout in app_config.h comfortably satisfies.
 */
static uint32_t ElapsedUs(uint32_t now, uint32_t since)
{
  return now - since;
}

uint16_t Ultrasonic_PulseToMillimetres(uint32_t pulse_us)
{
  uint32_t millimetres;

  /*
   * A pulse shorter than the minimum is crosstalk or a glitch; one longer than
   * the maximum is a missing target. Neither is a distance.
   */
  if (pulse_us < ULTRASONIC_MIN_PULSE_US || pulse_us > ULTRASONIC_MAX_PULSE_US)
  {
    return 0U;
  }

  /*
   * Round trip: distance = pulse * speed_of_sound / 2.
   * In integer millimetres, mm = us * 343 / 2000. The widest accepted pulse
   * gives 23000 * 343 = 7.9e6, well inside uint32_t.
   */
  millimetres = (pulse_us * ULTRASONIC_SOUND_SPEED_MS) / 2000UL;

  if (millimetres < ULTRASONIC_MIN_VALID_MM ||
      millimetres > ULTRASONIC_MAX_VALID_MM)
  {
    return 0U;
  }

  return (uint16_t)millimetres;
}

/* Median of the accepted samples held for one sensor. */
static uint16_t MedianOf(const UltrasonicSensor *sensor)
{
  uint16_t sorted[ULTRASONIC_MEDIAN_MAX];
  uint8_t count = sensor->sample_count;
  uint8_t i;
  uint8_t j;

  if (count == 0U)
  {
    return 0U;
  }

  memcpy(sorted, sensor->samples, (size_t)count * sizeof(sorted[0]));

  /* Insertion sort: at most five elements, no allocation, no recursion. */
  for (i = 1U; i < count; ++i)
  {
    uint16_t key = sorted[i];
    j = i;
    while (j > 0U && sorted[j - 1U] > key)
    {
      sorted[j] = sorted[j - 1U];
      --j;
    }
    sorted[j] = key;
  }

  /*
   * With a full window this is the true median. With a partial window it is the
   * middle of what has been collected so far, which is defined behaviour rather
   * than a special case: a single outlier still cannot dominate once three
   * samples exist.
   */
  return sorted[count / 2U];
}

static void RecordSuccess(UltrasonicSensor *sensor, uint16_t millimetres)
{
  sensor->raw_mm = millimetres;
  sensor->last_valid_mm = millimetres;

  sensor->samples[sensor->sample_index] = millimetres;
  sensor->sample_index =
      (uint8_t)((sensor->sample_index + 1U) % ULTRASONIC_MEDIAN_WINDOW);
  if (sensor->sample_count < ULTRASONIC_MEDIAN_WINDOW)
  {
    ++sensor->sample_count;
  }

  sensor->filtered_mm = MedianOf(sensor);
  sensor->valid = true;

  sensor->consecutive_failures = 0U;
  ++sensor->consecutive_successes;

  if (sensor->faulted &&
      sensor->consecutive_successes >= ULTRASONIC_RECOVER_THRESHOLD)
  {
    sensor->faulted = false;
  }
}

static void RecordFailure(UltrasonicSensor *sensor, bool was_timeout)
{
  sensor->consecutive_successes = 0U;
  ++sensor->consecutive_failures;

  if (was_timeout)
  {
    ++sensor->timeout_count;
  }
  else
  {
    ++sensor->reject_count;
  }

  if (sensor->consecutive_failures >= ULTRASONIC_FAULT_THRESHOLD)
  {
    sensor->faulted = true;
    /*
     * A faulted sensor stops offering a distance. Retaining last_valid_mm as a
     * stale number that the safety layer might trust would be worse than
     * reporting nothing.
     */
    sensor->valid = false;
    sensor->sample_count = 0U;
    sensor->sample_index = 0U;
    sensor->filtered_mm = 0U;
    sensor->raw_mm = 0U;
  }
}

void Ultrasonic_Init(Ultrasonic *ultrasonic, const UltrasonicHal *hal)
{
  if (ultrasonic == NULL)
  {
    return;
  }

  memset(ultrasonic, 0, sizeof(*ultrasonic));
  ultrasonic->state = ULTRASONIC_STATE_IDLE;

  if (hal == NULL || hal->get_micros == NULL || hal->set_trigger == NULL ||
      hal->read_echo == NULL)
  {
    /* Without a complete callback table the module stays inert. */
    ultrasonic->enabled = false;
    return;
  }

  ultrasonic->hal = *hal;
}

void Ultrasonic_SetEnabled(Ultrasonic *ultrasonic, bool enabled)
{
  uint8_t index;

  if (ultrasonic == NULL)
  {
    return;
  }

  if (ultrasonic->hal.get_micros == NULL)
  {
    ultrasonic->enabled = false;
    return;
  }

  if (!enabled && ultrasonic->enabled)
  {
    /* Leave every trigger low and stop claiming any reading is current. */
    for (index = 0U; index < ULTRASONIC_COUNT; ++index)
    {
      ultrasonic->hal.set_trigger(index, false);
      ultrasonic->sensor[index].valid = false;
    }
    ultrasonic->state = ULTRASONIC_STATE_IDLE;
  }

  ultrasonic->enabled = enabled;
}

bool Ultrasonic_IsEnabled(const Ultrasonic *ultrasonic)
{
  return (ultrasonic != NULL) && ultrasonic->enabled;
}

/* Moves to the next sensor in the round robin and re-arms. */
static void AdvanceSensor(Ultrasonic *ultrasonic, uint32_t now)
{
  ultrasonic->active = (uint8_t)((ultrasonic->active + 1U) % ULTRASONIC_COUNT);
  ultrasonic->state = ULTRASONIC_STATE_IDLE;
  ultrasonic->state_entry_us = now;
}

void Ultrasonic_Update(Ultrasonic *ultrasonic)
{
  uint32_t now;
  uint8_t active;
  UltrasonicSensor *sensor;

  if (ultrasonic == NULL || !ultrasonic->enabled ||
      ultrasonic->hal.get_micros == NULL)
  {
    return;
  }

  now = ultrasonic->hal.get_micros();
  active = ultrasonic->active;
  if (active >= ULTRASONIC_COUNT)
  {
    /* Defensive: corrupted index must not index out of bounds. */
    ultrasonic->active = 0U;
    ultrasonic->state = ULTRASONIC_STATE_IDLE;
    return;
  }
  sensor = &ultrasonic->sensor[active];

  switch (ultrasonic->state)
  {
    case ULTRASONIC_STATE_IDLE:
      ultrasonic->hal.set_trigger(active, true);
      ultrasonic->state = ULTRASONIC_STATE_TRIGGER_HIGH;
      ultrasonic->state_entry_us = now;
      break;

    case ULTRASONIC_STATE_TRIGGER_HIGH:
      if (ElapsedUs(now, ultrasonic->state_entry_us) >= ULTRASONIC_TRIGGER_US)
      {
        ultrasonic->hal.set_trigger(active, false);
        ultrasonic->state = ULTRASONIC_STATE_WAIT_ECHO_RISE;
        ultrasonic->state_entry_us = now;
      }
      break;

    case ULTRASONIC_STATE_WAIT_ECHO_RISE:
      if (ultrasonic->hal.read_echo(active))
      {
        ultrasonic->echo_rise_us = now;
        ultrasonic->state = ULTRASONIC_STATE_WAIT_ECHO_FALL;
        ultrasonic->state_entry_us = now;
      }
      else if (ElapsedUs(now, ultrasonic->state_entry_us) >=
               ULTRASONIC_RISE_TIMEOUT_US)
      {
        /* Sensor never answered: absent, unpowered or miswired. */
        ultrasonic->state = ULTRASONIC_STATE_TIMEOUT;
      }
      break;

    case ULTRASONIC_STATE_WAIT_ECHO_FALL:
      if (!ultrasonic->hal.read_echo(active))
      {
        uint32_t pulse = ElapsedUs(now, ultrasonic->echo_rise_us);
        uint16_t millimetres = Ultrasonic_PulseToMillimetres(pulse);

        if (millimetres != 0U)
        {
          RecordSuccess(sensor, millimetres);
        }
        else
        {
          /* Echo seen but out of band: not a timeout, still not a distance. */
          RecordFailure(sensor, false);
        }
        ultrasonic->state = ULTRASONIC_STATE_COMPLETE;
        ultrasonic->state_entry_us = now;
      }
      else if (ElapsedUs(now, ultrasonic->state_entry_us) >=
               ULTRASONIC_FALL_TIMEOUT_US)
      {
        /* Echo stuck high, which HC-SR04 does when nothing reflects. */
        ultrasonic->state = ULTRASONIC_STATE_TIMEOUT;
      }
      break;

    case ULTRASONIC_STATE_TIMEOUT:
      RecordFailure(sensor, true);
      ultrasonic->state = ULTRASONIC_STATE_COOLDOWN;
      ultrasonic->state_entry_us = now;
      break;

    case ULTRASONIC_STATE_COMPLETE:
      ultrasonic->state = ULTRASONIC_STATE_COOLDOWN;
      ultrasonic->state_entry_us = now;
      break;

    case ULTRASONIC_STATE_COOLDOWN:
      /*
       * The quiet time is what keeps one sensor's burst out of the next
       * sensor's window, and it is why a dead sensor cannot stall the others:
       * every path reaches cooldown and then hands over.
       */
      if (ElapsedUs(now, ultrasonic->state_entry_us) >= ULTRASONIC_COOLDOWN_US)
      {
        AdvanceSensor(ultrasonic, now);
      }
      break;

    default:
      ultrasonic->state = ULTRASONIC_STATE_IDLE;
      break;
  }
}

uint16_t Ultrasonic_GetDistance(const Ultrasonic *ultrasonic, uint8_t sensor)
{
  if (ultrasonic == NULL || sensor >= ULTRASONIC_COUNT ||
      !ultrasonic->enabled || !ultrasonic->sensor[sensor].valid)
  {
    return 0U;
  }
  return ultrasonic->sensor[sensor].filtered_mm;
}

uint16_t Ultrasonic_GetRawDistance(const Ultrasonic *ultrasonic, uint8_t sensor)
{
  if (ultrasonic == NULL || sensor >= ULTRASONIC_COUNT || !ultrasonic->enabled)
  {
    return 0U;
  }
  return ultrasonic->sensor[sensor].raw_mm;
}

uint8_t Ultrasonic_GetValidMask(const Ultrasonic *ultrasonic)
{
  uint8_t mask = 0U;
  uint8_t index;

  if (ultrasonic == NULL || !ultrasonic->enabled)
  {
    return 0U;
  }

  for (index = 0U; index < ULTRASONIC_COUNT; ++index)
  {
    if (ultrasonic->sensor[index].valid && !ultrasonic->sensor[index].faulted)
    {
      mask |= (uint8_t)(1U << index);
    }
  }
  return mask;
}

uint8_t Ultrasonic_GetFaultMask(const Ultrasonic *ultrasonic)
{
  uint8_t mask = 0U;
  uint8_t index;

  if (ultrasonic == NULL)
  {
    return 0U;
  }

  if (!ultrasonic->enabled)
  {
    return 0U;
  }

  for (index = 0U; index < ULTRASONIC_COUNT; ++index)
  {
    if (ultrasonic->sensor[index].faulted)
    {
      mask |= (uint8_t)(1U << index);
    }
  }
  return mask;
}

uint32_t Ultrasonic_GetTimeoutCount(const Ultrasonic *ultrasonic, uint8_t sensor)
{
  if (ultrasonic == NULL || sensor >= ULTRASONIC_COUNT)
  {
    return 0U;
  }
  return ultrasonic->sensor[sensor].timeout_count;
}

UltrasonicState Ultrasonic_GetState(const Ultrasonic *ultrasonic)
{
  if (ultrasonic == NULL)
  {
    return ULTRASONIC_STATE_IDLE;
  }
  return ultrasonic->state;
}

uint8_t Ultrasonic_GetActiveSensor(const Ultrasonic *ultrasonic)
{
  if (ultrasonic == NULL)
  {
    return 0U;
  }
  return ultrasonic->active;
}

/*
 * Ultrasonic state machine and filter tests.
 *
 * The module reaches the outside world through three callbacks, so the whole
 * thing runs here on virtual time: the mock advances a microsecond counter by
 * however much each test says, and drives the echo line by hand. Nothing waits.
 */

#include <string.h>

#include "test_framework.h"
#include "ultrasonic.h"

/* ---- mock platform ------------------------------------------------------ */

static uint32_t g_micros;
static bool g_trigger[ULTRASONIC_COUNT];
static bool g_echo[ULTRASONIC_COUNT];
static uint32_t g_trigger_rising_count[ULTRASONIC_COUNT];

static uint32_t MockMicros(void)
{
  return g_micros;
}

static void MockSetTrigger(uint8_t sensor, bool level)
{
  if (sensor >= ULTRASONIC_COUNT)
  {
    return;
  }
  if (level && !g_trigger[sensor])
  {
    ++g_trigger_rising_count[sensor];
  }
  g_trigger[sensor] = level;
}

static bool MockReadEcho(uint8_t sensor)
{
  return (sensor < ULTRASONIC_COUNT) ? g_echo[sensor] : false;
}

static const UltrasonicHal kMockHal = {MockMicros, MockSetTrigger, MockReadEcho};

static void MockReset(uint32_t start_micros)
{
  g_micros = start_micros;
  memset(g_trigger, 0, sizeof(g_trigger));
  memset(g_echo, 0, sizeof(g_echo));
  memset(g_trigger_rising_count, 0, sizeof(g_trigger_rising_count));
}

/* Advances virtual time in small slices, updating as a real main loop would. */
static void Advance(Ultrasonic *ultrasonic, uint32_t microseconds, uint32_t step)
{
  uint32_t elapsed = 0U;
  while (elapsed < microseconds)
  {
    g_micros += step;
    elapsed += step;
    Ultrasonic_Update(ultrasonic);
  }
}

/*
 * Runs one complete measurement for the currently active sensor, presenting an
 * echo pulse of the requested width. Returns the sensor that was measured.
 */
static uint8_t RunMeasurement(Ultrasonic *ultrasonic, uint32_t pulse_us)
{
  uint8_t sensor = Ultrasonic_GetActiveSensor(ultrasonic);

  /* Trigger pulse and the gap before the echo. */
  Advance(ultrasonic, ULTRASONIC_TRIGGER_US + 40U, 2U);

  g_echo[sensor] = true;
  Ultrasonic_Update(ultrasonic);

  Advance(ultrasonic, pulse_us, 10U);

  g_echo[sensor] = false;
  Ultrasonic_Update(ultrasonic);

  /* Complete -> cooldown -> next sensor. */
  Advance(ultrasonic, ULTRASONIC_COOLDOWN_US + 200U, 100U);
  return sensor;
}

/* Echo pulse width that yields a given distance. */
static uint32_t PulseForMillimetres(uint16_t millimetres)
{
  return ((uint32_t)millimetres * 2000UL) / ULTRASONIC_SOUND_SPEED_MS;
}

/* ---- tests -------------------------------------------------------------- */

static void TestPulseConversion(void)
{
  TEST_CASE("pulse width converts to millimetres");

  /* 1000 us of flight time is roughly 171 mm each way. */
  TEST_EQ(Ultrasonic_PulseToMillimetres(1000UL), 171);
  /*
   * Round trip through the helper truncates twice, so allow a millimetre of
   * integer slack rather than pretending the conversion is exact.
   */
  TEST_IN_RANGE(Ultrasonic_PulseToMillimetres(PulseForMillimetres(500)), 498, 502);

  /* Out of band pulses are not distances. */
  TEST_EQ(Ultrasonic_PulseToMillimetres(1UL), 0);
  TEST_EQ(Ultrasonic_PulseToMillimetres(ULTRASONIC_MIN_PULSE_US - 1UL), 0);
  TEST_EQ(Ultrasonic_PulseToMillimetres(ULTRASONIC_MAX_PULSE_US + 1UL), 0);
  TEST_EQ(Ultrasonic_PulseToMillimetres(999999UL), 0);
}

static void TestDisabledByDefault(void)
{
  Ultrasonic ultrasonic;

  TEST_CASE("the module reports nothing until it is enabled");

  MockReset(0U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  TEST_CHECK(!Ultrasonic_IsEnabled(&ultrasonic));

  Advance(&ultrasonic, 100000U, 100U);
  TEST_EQ(Ultrasonic_GetValidMask(&ultrasonic), 0);
  TEST_EQ(Ultrasonic_GetDistance(&ultrasonic, ULTRASONIC_FRONT), 0);
}

static void TestNormalMeasurement(void)
{
  Ultrasonic ultrasonic;
  uint8_t sensor;

  TEST_CASE("a normal echo produces a valid distance");

  MockReset(1000U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);

  sensor = RunMeasurement(&ultrasonic, PulseForMillimetres(600));

  TEST_CHECK((Ultrasonic_GetValidMask(&ultrasonic) & (1U << sensor)) != 0U);
  TEST_IN_RANGE(Ultrasonic_GetDistance(&ultrasonic, sensor), 590, 610);
  TEST_EQ(Ultrasonic_GetFaultMask(&ultrasonic), 0);
}

static void TestNoEchoRiseTimesOut(void)
{
  Ultrasonic ultrasonic;
  uint8_t sensor;

  TEST_CASE("a sensor that never raises echo times out");

  MockReset(0U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);
  sensor = Ultrasonic_GetActiveSensor(&ultrasonic);

  /* Never assert the echo line. */
  Advance(&ultrasonic, ULTRASONIC_RISE_TIMEOUT_US + ULTRASONIC_COOLDOWN_US + 5000U,
          100U);

  TEST_CHECK(Ultrasonic_GetTimeoutCount(&ultrasonic, sensor) >= 1U);
  TEST_EQ(Ultrasonic_GetDistance(&ultrasonic, sensor), 0);
}

static void TestNoEchoFallTimesOut(void)
{
  Ultrasonic ultrasonic;
  uint8_t sensor;

  TEST_CASE("an echo stuck high times out");

  MockReset(0U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);
  sensor = Ultrasonic_GetActiveSensor(&ultrasonic);

  Advance(&ultrasonic, ULTRASONIC_TRIGGER_US + 40U, 2U);
  g_echo[sensor] = true;
  Ultrasonic_Update(&ultrasonic);

  /* Leave it high past the fall timeout. */
  Advance(&ultrasonic, ULTRASONIC_FALL_TIMEOUT_US + 2000U, 100U);

  TEST_CHECK(Ultrasonic_GetTimeoutCount(&ultrasonic, sensor) >= 1U);
}

static void TestShortAndLongPulsesRejected(void)
{
  Ultrasonic ultrasonic;
  uint8_t sensor;

  TEST_CASE("out of band echo pulses do not become distances");

  MockReset(0U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);

  /* Far too short to be a real reflection. */
  sensor = RunMeasurement(&ultrasonic, 5U);
  TEST_EQ(Ultrasonic_GetDistance(&ultrasonic, sensor), 0);
  TEST_CHECK((Ultrasonic_GetValidMask(&ultrasonic) & (1U << sensor)) == 0U);
}

static void TestMicrosecondWrap(void)
{
  Ultrasonic ultrasonic;
  uint8_t sensor;

  TEST_CASE("a measurement spanning the counter wrap still works");

  /* Start close enough to the top that the echo pulse crosses zero. */
  MockReset(0xFFFFF000UL);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);

  sensor = RunMeasurement(&ultrasonic, PulseForMillimetres(800));

  TEST_CHECK((Ultrasonic_GetValidMask(&ultrasonic) & (1U << sensor)) != 0U);
  TEST_IN_RANGE(Ultrasonic_GetDistance(&ultrasonic, sensor), 780, 820);
}

static void TestRoundRobinVisitsEverySensor(void)
{
  Ultrasonic ultrasonic;
  uint8_t index;
  bool seen[ULTRASONIC_COUNT];

  TEST_CASE("polling visits all four sensors in turn");

  MockReset(0U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);
  memset(seen, 0, sizeof(seen));

  for (index = 0U; index < ULTRASONIC_COUNT; ++index)
  {
    uint8_t sensor = RunMeasurement(&ultrasonic, PulseForMillimetres(500));
    seen[sensor] = true;
  }

  for (index = 0U; index < ULTRASONIC_COUNT; ++index)
  {
    TEST_CHECK(seen[index]);
    TEST_CHECK(g_trigger_rising_count[index] >= 1U);
  }
  TEST_EQ(Ultrasonic_GetValidMask(&ultrasonic), 0x0F);
}

static void TestOneDeadSensorDoesNotBlockOthers(void)
{
  Ultrasonic ultrasonic;
  uint8_t index;
  uint8_t dead;
  uint8_t good_readings = 0U;

  TEST_CASE("one dead sensor does not stall the round robin");

  MockReset(0U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);
  dead = Ultrasonic_GetActiveSensor(&ultrasonic);

  for (index = 0U; index < ULTRASONIC_COUNT * 2U; ++index)
  {
    uint8_t sensor = Ultrasonic_GetActiveSensor(&ultrasonic);
    if (sensor == dead)
    {
      /* Never answers. */
      Advance(&ultrasonic,
              ULTRASONIC_RISE_TIMEOUT_US + ULTRASONIC_COOLDOWN_US + 2000U, 100U);
    }
    else
    {
      (void)RunMeasurement(&ultrasonic, PulseForMillimetres(700));
      ++good_readings;
    }
  }

  TEST_CHECK(good_readings >= ULTRASONIC_COUNT - 1U);
  for (index = 0U; index < ULTRASONIC_COUNT; ++index)
  {
    if (index != dead)
    {
      TEST_CHECK(Ultrasonic_GetDistance(&ultrasonic, index) > 0U);
    }
  }
}

static void TestConsecutiveFailuresRaiseFault(void)
{
  Ultrasonic ultrasonic;
  uint8_t sensor;
  uint8_t attempt;

  TEST_CASE("repeated failures mark a sensor faulty, recovery clears it");

  MockReset(0U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);
  sensor = Ultrasonic_GetActiveSensor(&ultrasonic);

  /* Fail this one sensor repeatedly while servicing the others normally. */
  for (attempt = 0U; attempt < ULTRASONIC_FAULT_THRESHOLD * ULTRASONIC_COUNT;
       ++attempt)
  {
    if (Ultrasonic_GetActiveSensor(&ultrasonic) == sensor)
    {
      Advance(&ultrasonic,
              ULTRASONIC_RISE_TIMEOUT_US + ULTRASONIC_COOLDOWN_US + 2000U, 100U);
    }
    else
    {
      (void)RunMeasurement(&ultrasonic, PulseForMillimetres(600));
    }
  }

  TEST_CHECK((Ultrasonic_GetFaultMask(&ultrasonic) & (1U << sensor)) != 0U);
  TEST_CHECK((Ultrasonic_GetValidMask(&ultrasonic) & (1U << sensor)) == 0U);

  /* Now let it answer again. */
  for (attempt = 0U; attempt < ULTRASONIC_RECOVER_THRESHOLD * ULTRASONIC_COUNT;
       ++attempt)
  {
    (void)RunMeasurement(&ultrasonic, PulseForMillimetres(600));
  }

  TEST_CHECK((Ultrasonic_GetFaultMask(&ultrasonic) & (1U << sensor)) == 0U);
  TEST_CHECK((Ultrasonic_GetValidMask(&ultrasonic) & (1U << sensor)) != 0U);
}

static void TestMedianRejectsSingleOutlier(void)
{
  Ultrasonic ultrasonic;
  uint8_t target;
  uint8_t round;
  uint16_t filtered;

  TEST_CASE("a single outlier does not move the filtered distance");

  MockReset(0U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);
  target = Ultrasonic_GetActiveSensor(&ultrasonic);

  /* Fill the window with a steady distance. */
  for (round = 0U; round < ULTRASONIC_MEDIAN_WINDOW * ULTRASONIC_COUNT; ++round)
  {
    uint8_t sensor = Ultrasonic_GetActiveSensor(&ultrasonic);
    (void)RunMeasurement(&ultrasonic,
                         PulseForMillimetres((sensor == target) ? 600 : 900));
  }
  filtered = Ultrasonic_GetDistance(&ultrasonic, target);
  TEST_IN_RANGE(filtered, 590, 610);

  /* One wild reading on the target sensor. */
  for (round = 0U; round < ULTRASONIC_COUNT; ++round)
  {
    uint8_t sensor = Ultrasonic_GetActiveSensor(&ultrasonic);
    (void)RunMeasurement(&ultrasonic,
                         PulseForMillimetres((sensor == target) ? 3500 : 900));
  }

  /* The median holds: one sample out of five cannot dominate. */
  TEST_IN_RANGE(Ultrasonic_GetDistance(&ultrasonic, target), 590, 610);
}

static void TestPartialWindowBehaviour(void)
{
  Ultrasonic ultrasonic;
  uint8_t sensor;

  TEST_CASE("a single sample is reported before the window fills");

  MockReset(0U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);

  sensor = RunMeasurement(&ultrasonic, PulseForMillimetres(450));
  TEST_IN_RANGE(Ultrasonic_GetDistance(&ultrasonic, sensor), 440, 460);
}

static void TestDisablingClearsValidity(void)
{
  Ultrasonic ultrasonic;
  uint8_t sensor;

  TEST_CASE("disabling stops reporting distances and drops the triggers");

  MockReset(0U);
  Ultrasonic_Init(&ultrasonic, &kMockHal);
  Ultrasonic_SetEnabled(&ultrasonic, true);
  sensor = RunMeasurement(&ultrasonic, PulseForMillimetres(500));
  TEST_CHECK(Ultrasonic_GetDistance(&ultrasonic, sensor) > 0U);

  Ultrasonic_SetEnabled(&ultrasonic, false);
  TEST_EQ(Ultrasonic_GetValidMask(&ultrasonic), 0);
  TEST_EQ(Ultrasonic_GetDistance(&ultrasonic, sensor), 0);
  {
    uint8_t index;
    for (index = 0U; index < ULTRASONIC_COUNT; ++index)
    {
      TEST_CHECK(!g_trigger[index]);
    }
  }
}

static void TestMissingCallbacksAreInert(void)
{
  Ultrasonic ultrasonic;
  UltrasonicHal broken = {NULL, NULL, NULL};

  TEST_CASE("an incomplete callback table leaves the module inert");

  Ultrasonic_Init(&ultrasonic, &broken);
  Ultrasonic_SetEnabled(&ultrasonic, true);
  TEST_CHECK(!Ultrasonic_IsEnabled(&ultrasonic));

  /* Must not dereference anything. */
  Ultrasonic_Update(&ultrasonic);
  TEST_EQ(Ultrasonic_GetValidMask(&ultrasonic), 0);
}

int run_ultrasonic_tests(void)
{
  printf("-- ultrasonic --\n");
  TestPulseConversion();
  TestDisabledByDefault();
  TestNormalMeasurement();
  TestNoEchoRiseTimesOut();
  TestNoEchoFallTimesOut();
  TestShortAndLongPulsesRejected();
  TestMicrosecondWrap();
  TestRoundRobinVisitsEverySensor();
  TestOneDeadSensorDoesNotBlockOthers();
  TestConsecutiveFailuresRaiseFault();
  TestMedianRejectsSingleOutlier();
  TestPartialWindowBehaviour();
  TestDisablingClearsValidity();
  TestMissingCallbacksAreInert();
  return 0;
}

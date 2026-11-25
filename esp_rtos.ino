/*
 * ESP32 dual ultrasonic reader - 100 Hz per sensor, one sensor per core,
 * with on-device logging to verify the rate and the dual-core operation.
 *
 * Pins/wiring taken from github.com/rajmehta28599/Ultrasonic_TestCOde:
 *   HC-SR04  : TRIG = GPIO12, ECHO = GPIO14   (core 0)
 *   AJ-SR04M : TRIG = GPIO26, ECHO = GPIO25   (core 1)
 *   VCC = VIN (5 V), GND = GND
 *   ECHO lines MUST go through a 1k/2k voltage divider (ESP32 is 3.3 V only).
 *   GPIO12 is a boot strapping pin: if boot fails, move HC TRIG to GPIO27/33.
 *
 * Board: ESP32 Dev Module. Serial monitor: 921600 baud.
 *
 * Output
 *   Data  (only if PRINT_DATA = true), one line per measurement:
 *       HC,<time_us>,<distance_cm>      (-1.00 = no echo / out of range)
 *   Stats (every STATS_EVERY samples = 1 s per sensor), lines start with '#':
 *       # STATS HC core=0 rate=100.00Hz [OK] period_us(min/avg/max)=... busy_us(avg/max)=... echo=98/100 late=0
 *
 * How to read the stats:
 *   core      core the task is really running on (xPortGetCoreID) -> must differ
 *   rate      achieved samples/s -> must be ~100 on BOTH sensors in the same window
 *   period    time between cycle starts; avg ~10000 us, min/max show jitter
 *   busy      time spent inside one measurement. Each sensor can use up to ~9.5 ms
 *             of every 10 ms, so both holding 100 Hz together is only possible
 *             if they run in parallel on separate cores.
 *   late      cycles whose period exceeded 10 ms by more than LATE_TOL_US
 */

#include <Arduino.h>

// ---------------- Options ----------------
constexpr bool PRINT_DATA = false;   // true = also stream every measurement

// ---------------- Pins ----------------
constexpr uint8_t HC_TRIG = 12;
constexpr uint8_t HC_ECHO = 14;
constexpr uint8_t AJ_TRIG = 26;
constexpr uint8_t AJ_ECHO = 25;

// ---------------- Timing ----------------
constexpr TickType_t PERIOD_TICKS   = 10;     // 10 ticks * 1 ms = 10 ms -> 100 Hz
constexpr uint32_t   PERIOD_US      = 10000;
constexpr uint32_t   ECHO_WINDOW_US = 9500;   // must be < 10 ms period
// 9.5 ms round trip at 343 m/s -> max range ~163 cm (that's the 100 Hz limit)
constexpr uint32_t   LATE_TOL_US    = 1500;   // period > 10 ms + this = "late"
constexpr uint32_t   STATS_EVERY    = 100;    // samples per stats line (~1 s)

constexpr float SPEED_CM_PER_US = 0.0343f;    // ~20 C; adjust for temperature

struct Sensor {
  const char *name;
  uint8_t trig;
  uint8_t echo;
  float hist[3];  // last 3 raw readings for median filter
  uint8_t idx;

  // --- statistics for the current window ---
  bool     haveLast;
  uint32_t lastStartUs;
  uint32_t windowStartUs;
  uint32_t count;
  uint32_t valid;
  uint32_t late;
  uint32_t pMin, pMax;
  uint64_t pSum;
  uint32_t pCount;
  uint32_t busyMax;
  uint64_t busySum;
};

Sensor hc = {"HC", HC_TRIG, HC_ECHO, {-1, -1, -1}, 0};
Sensor aj = {"AJ", AJ_TRIG, AJ_ECHO, {-1, -1, -1}, 0};

static void resetWindow(Sensor &s, uint32_t nowUs) {
  s.windowStartUs = nowUs;
  s.count = s.valid = s.late = 0;
  s.pMin = UINT32_MAX;
  s.pMax = 0;
  s.pSum = 0;
  s.pCount = 0;
  s.busyMax = 0;
  s.busySum = 0;
}

// Single raw measurement. Returns cm, or -1 on timeout.
static float measureCm(const Sensor &s) {
  digitalWrite(s.trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(s.trig, LOW);

  const uint32_t t0 = micros();

  // wait for echo to go high
  while (!digitalRead(s.echo)) {
    if (micros() - t0 > ECHO_WINDOW_US) return -1.0f;
  }
  const uint32_t rise = micros();

  // wait for echo to go low
  while (digitalRead(s.echo)) {
    if (micros() - t0 > ECHO_WINDOW_US) return -1.0f;
  }
  const uint32_t width = micros() - rise;

  return width * SPEED_CM_PER_US * 0.5f;
}

// Median of 3 - removes single-sample spikes/dropouts without adding lag.
static float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

static void printStats(Sensor &s, uint32_t nowUs) {
  const uint32_t elapsed = nowUs - s.windowStartUs;
  const float rate = elapsed ? (s.count * 1000000.0f / elapsed) : 0.0f;
  const uint32_t pAvg = s.pCount ? (uint32_t)(s.pSum / s.pCount) : 0;
  const uint32_t bAvg = s.count ? (uint32_t)(s.busySum / s.count) : 0;
  const bool ok = (rate >= 99.0f && rate <= 101.0f);

  Serial.printf("# STATS %s core=%d rate=%.2fHz [%s] period_us(min/avg/max)=%lu/%lu/%lu "
                "busy_us(avg/max)=%lu/%lu echo=%lu/%lu late=%lu\n",
                s.name, (int)xPortGetCoreID(), rate, ok ? "OK" : "LOW/HIGH",
                (unsigned long)(s.pCount ? s.pMin : 0), (unsigned long)pAvg,
                (unsigned long)s.pMax, (unsigned long)bAvg, (unsigned long)s.busyMax,
                (unsigned long)s.valid, (unsigned long)s.count, (unsigned long)s.late);
}

static void sensorTask(void *arg) {
  Sensor *s = static_cast<Sensor *>(arg);

  pinMode(s->trig, OUTPUT);
  digitalWrite(s->trig, LOW);
  pinMode(s->echo, INPUT);

  resetWindow(*s, micros());
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    const uint32_t cycleStart = micros();

    // --- period / jitter bookkeeping ---
    if (s->haveLast) {
      const uint32_t period = cycleStart - s->lastStartUs;
      if (period < s->pMin) s->pMin = period;
      if (period > s->pMax) s->pMax = period;
      s->pSum += period;
      s->pCount++;
      if (period > PERIOD_US + LATE_TOL_US) s->late++;
    }
    s->haveLast = true;
    s->lastStartUs = cycleStart;

    // --- measurement ---
    const float raw = measureCm(*s);
    const uint32_t busy = micros() - cycleStart;
    s->busySum += busy;
    if (busy > s->busyMax) s->busyMax = busy;

    s->hist[s->idx] = raw;
    s->idx = (s->idx + 1) % 3;

    float out = median3(s->hist[0], s->hist[1], s->hist[2]);
    if (out < 0) out = -1.0f;

    s->count++;
    if (raw >= 0) s->valid++;

    // one call -> one atomic line, safe with both cores printing
    if (PRINT_DATA) {
      Serial.printf("%s,%lu,%.2f\n", s->name, (unsigned long)cycleStart, out);
    }

    // --- periodic stats ---
    if (s->count >= STATS_EVERY) {
      const uint32_t now = micros();
      printStats(*s, now);
      resetWindow(*s, now);
    }

    // fixed-rate loop: exactly 100 Hz regardless of how long the echo took
    vTaskDelayUntil(&lastWake, PERIOD_TICKS);
  }
}

void setup() {
  Serial.begin(921600);
  delay(300);
  Serial.println("# start: HC-SR04 on core0, AJ-SR04M on core1, target 100 Hz each");

  // args: function, name, stack, param, priority, handle, core
  xTaskCreatePinnedToCore(sensorTask, "hc_task", 6144, &hc, 5, nullptr, 0);
  xTaskCreatePinnedToCore(sensorTask, "aj_task", 6144, &aj, 5, nullptr, 1);
}

void loop() {
  vTaskDelete(nullptr);  // nothing to do here; the two tasks do all the work
}
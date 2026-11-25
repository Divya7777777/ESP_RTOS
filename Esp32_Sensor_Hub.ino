/*
 * ESP32 Sensor + LED Hub
 * ----------------------------------------------------------------------------
 * Replaces: the sensor side of the Teensy sketch + ESP32 #1 (data) + ESP32 #2 (LED/WCD).
 * The ZLAC motor driver now talks to the Jetson directly, so motor control, PID and
 * encoders are NOT in this firmware.
 *
 * Libraries (Arduino Library Manager):
 *   - "BMI088" by Bolder Flight Systems   (same lib/calls as the Teensy sketch)
 *   - "Mcp320x" by labfruits              (same lib/calls as the Teensy sketch)
 *   - "FastLED"
 * Board: ESP32 Dev Module (arduino-esp32 core 2.x or 3.x). Not compiled by the author:
 * build it once and fix any library-version differences.
 *
 * ---------------------------------------------------------------------------
 * USB SERIAL PROTOCOL  (921600 baud, text lines ending in '\n')
 * ---------------------------------------------------------------------------
 * ESP -> Jetson
 *   d,<seq>,<t_ms>,<yaw>,<batt%>,<estop>,<charging>,<volts>,<amps>,
 *     <us1_mm>,<us2_mm>,<us3_mm>,<us4_mm>,<cliffL>,<cliffR>,<ir_bits>
 *       yaw       : (deg + 180) * 100  (0..36000), 44444 = gyro error  (same as Teensy)
 *       estop     : 1 = released, 0 = pressed                          (same as Teensy)
 *       charging  : 1 = battery current > 0.3 A                        (same as Teensy)
 *       us*_mm    : filtered distance in mm, -1 = no echo / out of range
 *       cliff*    : 1 = cliff detected
 *       ir_bits   : bit i = raw level of IR_PINS[i]
 *   s,estop | s,estart | s,stop | s,charger_on | s,charger_off     status events
 *   w,<mac>,<text or 0xHEX>                                          message from WCD
 *   ack,<cmd>                                                        command accepted
 *   # ...                                                            debug / stats (ignore)
 *
 * Jetson -> ESP
 *   led,<tray 0-3>,<pattern 0-5>,<r>,<g>,<b>    tray 0 = all trays
 *        pattern: 0 off, 1 solid, 2 blink, 3 breathe, 4 chase, 5 rainbow
 *   w,<text>                                      send text to the WCD over ESP-NOW
 *   r                                             reset yaw to 0
 * ---------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <esp_arduino_version.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_now.h>
#include <driver/gpio.h>
#include <Mcp320x.h>
#include "BMI088.h"
#include <FastLED.h>

// ============================== Options =====================================
constexpr uint32_t SERIAL_BAUD     = 921600;
constexpr uint32_t SENSOR_RATE_HZ  = 100;     // packet rate to Jetson: 50 or 100
constexpr bool     STATS_ENABLED   = true;    // '#' STATS line every STATS_PERIOD_MS
constexpr uint32_t STATS_PERIOD_MS = 2000;

// ============================== Pins (classic ESP32 DevKit) =================
// I2C BMI088 (kept on the default pins: the BMI088 library calls Wire.begin() itself)
constexpr uint8_t PIN_I2C_SDA = 21, PIN_I2C_SCL = 22;
// SPI MCP3208
constexpr uint8_t PIN_SPI_SCK = 18, PIN_SPI_MISO = 19, PIN_SPI_MOSI = 23, PIN_ADC_CS = 5;
// Power / safety inputs (pins with internal pull-ups)
constexpr uint8_t PIN_ESTOP_FB   = 32;   // 1 = released, 0 = pressed
constexpr uint8_t PIN_MAIN_SW_FB = 33;
constexpr uint8_t PIN_MAIN_RELAY = 16;
// Ultrasonics 1-2 are the pins from the Ultrasonic_TestCOde repo; 3-4 are new (assumed HC-SR04
// type, 5 V + 1k/2k divider on ECHO). GPIO36/39 are input-only: fine for ECHO.
const uint8_t US_TRIG[4] = {12, 26, 27, 4};
const uint8_t US_ECHO[4] = {14, 25, 36, 39};
// Which of the two time slots each sensor fires in. Sensors in the same slot fire together
// (each sensor then runs at 50 Hz). Put sensors that face each other in DIFFERENT slots.
const uint8_t US_SLOT[4] = {0, 1, 0, 1};
// Cliff sensors (GPIO34/35 are input-only with NO internal pull-up)
constexpr uint8_t PIN_CLIFF_L = 34, PIN_CLIFF_R = 35;
constexpr int     CLIFF_ACTIVE_LEVEL = LOW;   // level that means "cliff"
// IR docking array - PLACEHOLDER: set to the real pins / interface of your array
const uint8_t IR_PINS[] = {17};
// WS2812B strips (3 trays x 100 LEDs)
constexpr uint8_t PIN_LED1 = 13, PIN_LED2 = 15, PIN_LED3 = 2;

// ============================== Constants ===================================
// ADC / power (same values and maths as the Teensy sketch)
constexpr uint16_t ADC_VREF_MV = 5000;
// ESTOP
constexpr uint32_t ESTOP_DEBOUNCE_MS = 50;
// Gyro
constexpr int GYRO_CAL_SAMPLES = 200;
#define TO_DEG(x) ((x) * 57.2957795131f)
// Ultrasonic
constexpr TickType_t US_SLOT_TICKS = 10;        // 10 ms slot (1 ms FreeRTOS tick)
constexpr float      US_MM_PER_US  = 0.1715f;   // round trip, ~20 C
constexpr int        US_MIN_MM     = 20;
constexpr int        US_MAX_MM     = 1600;      // ~9.5 ms echo window
// Fast loop
constexpr TickType_t FAST_PERIOD_TICKS = 5;     // 200 Hz (same as Teensy LOOP_RATE)
constexpr uint32_t   FAST_PERIOD_US    = 5000;
// LEDs
constexpr uint16_t LEDS_PER_TRAY   = 100;
constexpr uint8_t  LED_BRIGHTNESS  = 80;        // 0-255 global limit
constexpr uint16_t LED_MAX_MA      = 3000;      // set to what your 5 V supply can give
// ESP-NOW / WCD
uint8_t WCD_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};   // broadcast; put the WCD MAC here
constexpr uint8_t WCD_MAX_PAYLOAD = 200;

// ============================== Shared state ================================
struct SensorState {
  float    yaw_rad;
  bool     gyro_ok;
  float    batt_pct, volts, amps;
  bool     charging;
  int      estop;               // 1 released, 0 pressed
  int16_t  us_mm[4];
  uint8_t  cliffL, cliffR, ir_bits;
};
SensorState S;
SemaphoreHandle_t stateMtx;

struct StatusMsg { char s[40]; };
QueueHandle_t statusQ;

struct WcdMsg { uint8_t mac[6]; uint8_t len; uint8_t data[WCD_MAX_PAYLOAD]; };
QueueHandle_t wcdQ;

struct TrayCmd { uint8_t pattern, r, g, b; };
TrayCmd tray[3];
volatile bool trayDirty = true;
portMUX_TYPE trayMux = portMUX_INITIALIZER_UNLOCKED;
CRGB leds[3][LEDS_PER_TRAY];

// stats (written by tasks, read by comms task; racy reads are fine for logging)
volatile uint32_t fastCount = 0, fastMaxUs = 0, fastLate = 0;
volatile uint32_t txCount = 0, txDrops = 0, cmdCount = 0, wcdRx = 0;
volatile uint32_t usOk[4] = {0}, usN[4] = {0};

// ============================== Helpers =====================================
static void pushStatus(const char *s) {
  StatusMsg m;
  strlcpy(m.s, s, sizeof(m.s));
  xQueueSend(statusQ, &m, 0);   // drop if full; never block a sensor task
}

static float median3(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ============================== E-stop / switches ===========================
static int estopLevel = 1;                 // debounced, 1 = released
static uint32_t estopReleasedMs = 0;
static bool mainRelayCut = false;          // "flag" in the Teensy sketch

static void updateEstop() {
  static int lastRaw = HIGH;
  static uint32_t since = 0;
  const int raw = digitalRead(PIN_ESTOP_FB);
  if (raw != lastRaw) {
    lastRaw = raw;
    since = millis();
  } else if (raw != estopLevel && millis() - since >= ESTOP_DEBOUNCE_MS) {
    estopLevel = raw;
  }
}

// Same events the Teensy sent to the NUC: "estop" every 5 s while pressed (>200 ms), "estart" on release.
static void estopStatusStep() {
  static uint32_t lastSent = 0;
  static bool sentOnce = false, stopFlag = false;
  if (estopLevel == 1) {
    estopReleasedMs = millis();
    if (stopFlag) { pushStatus("s,estart"); stopFlag = false; }
    sentOnce = false;
  } else if (millis() - estopReleasedMs > 200) {
    if (!sentOnce || millis() - lastSent >= 5000) {
      pushStatus("s,estop");
      lastSent = millis();
      sentOnce = true;
      stopFlag = true;
    }
  }
}

// Main-switch-off sequence: 5x "stop" to the Jetson (200 ms apart), then cut the relay after 15 s.
static void smartSwitchStep() {
  static enum { IDLE, SENDING_STOP, WAITING } st = IDLE;
  static uint32_t ts = 0;
  static int n = 0;
  switch (st) {
    case IDLE:
      if (digitalRead(PIN_MAIN_SW_FB) == 1 && estopLevel == 0) {
        st = SENDING_STOP; ts = millis(); n = 0;
      }
      break;
    case SENDING_STOP:
      if (millis() - ts >= 200) {
        pushStatus("s,stop");
        n++; ts = millis();
        if (n >= 5) st = WAITING;
      }
      break;
    case WAITING:
      if (millis() - ts >= 15000) {
        digitalWrite(PIN_MAIN_RELAY, LOW);
        mainRelayCut = true;
        st = IDLE;
      }
      break;
  }
}

// ============================== Gyro (BMI088) ===============================
Bmi088Gyro gyro(Wire, 0x69);
static float yaw2 = 0.0f, gyroBiasZ = 0.0f;
static bool gyroCalibrated = false;

static bool gyroInit() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  int status = -1;
  for (int i = 0; i < 5 && status < 0; i++) {
    status = gyro.begin();
    if (status < 0) { Serial.println("# BMI088 init failed, retrying"); delay(500); }
  }
  if (status < 0) return false;
  gyro.setOdr(Bmi088Gyro::ODR_200HZ_BW_64HZ);
  delay(500);
  float sum = 0.0f;                         // calibrate: keep the robot still
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    gyro.readSensor();
    sum += -gyro.getGyroZ_rads();
    delay(10);
  }
  gyroBiasZ = sum / GYRO_CAL_SAMPLES;
  gyroCalibrated = true;
  Serial.printf("# gyro bias z: %.6f\n", gyroBiasZ);
  return true;
}

static void gyroRead() {
  if (!gyroCalibrated) return;
  static uint32_t lastUs = 0;
  gyro.readSensor();
  float gz = -gyro.getGyroZ_rads() - gyroBiasZ;
  const uint32_t now = micros();
  if (lastUs == 0) { lastUs = now; return; }
  const float dt = (now - lastUs) / 1000000.0f;
  lastUs = now;
  if (dt > 1.0f) return;                    // timing broke: skip integration
  if (fabsf(gz) < 0.001f) gz = 0.0f;        // deadband
  yaw2 += gz * dt;
  while (yaw2 > M_PI)  yaw2 -= 2.0f * M_PI;
  while (yaw2 < -M_PI) yaw2 += 2.0f * M_PI;
  if (isnan(yaw2) || isinf(yaw2)) yaw2 = 0.0f;
}

// ============================== Power (MCP3208) =============================
MCP3208 adc(ADC_VREF_MV, PIN_ADC_CS);
static float batCurrentFiltered = 0, batCurrentSum = 0;
static float v24Filtered = 0, v24Sum = 0;
static float battPct = 0;
static bool charging = false, prevCharging = false;

static void powerMonitor() {
  const uint16_t rawCs   = adc.read(MCP3208::Channel::SINGLE_6);
  const uint16_t volCs   = adc.toAnalog(rawCs);
  const uint16_t rawRef  = adc.read(MCP3208::Channel::SINGLE_7);
  const uint16_t volRef  = adc.toAnalog(rawRef);
  const uint16_t rawVd   = adc.read(MCP3208::Channel::SINGLE_4);
  const uint16_t volVd   = adc.toAnalog(rawVd);

  const float batCurrent = (float((int)volRef - (int)volCs) / 2000.0f) * 25.0f;
  batCurrentSum = batCurrentSum + batCurrent - batCurrentFiltered;
  batCurrentFiltered = batCurrentSum / 200.0f;

  const float v24 = float(volVd) / 1000.0f;
  v24Sum = v24Sum + v24 - v24Filtered;
  v24Filtered = v24Sum / 200.0f;

  battPct = (((v24Filtered * 6.25f) - 20.0f) * 100.0f) / (24.0f - 20.0f);
  if (battPct > 100.0f) battPct = 100.0f;
  if (battPct < 0.0f)   battPct = 0.0f;

  charging = (batCurrentFiltered > 0.3f);   // 0.3 A threshold avoids noise triggers
  if (charging && !prevCharging)  pushStatus("s,charger_on");
  if (!charging && prevCharging)  pushStatus("s,charger_off");
  prevCharging = charging;
}

// ============================== Ultrasonics =================================
struct UsChan {
  uint8_t trig, echo;
  volatile uint32_t rise, fall;
  volatile bool gotRise, gotFall;
  float hist[3];
  uint8_t idx;
};
UsChan us[4];

static void ARDUINO_ISR_ATTR usIsr(void *arg) {
  UsChan *c = static_cast<UsChan *>(arg);
  const uint32_t t = micros();
  if (gpio_get_level((gpio_num_t)c->echo)) {
    c->rise = t; c->gotRise = true;
  } else if (c->gotRise) {
    c->fall = t; c->gotFall = true;
  }
}

static int16_t usFinalize(UsChan &c) {
  float mm = -1.0f;
  if (c.gotRise && c.gotFall) {
    const uint32_t w = c.fall - c.rise;
    const float d = w * US_MM_PER_US;
    if (d >= US_MIN_MM && d <= US_MAX_MM) mm = d;
  }
  c.hist[c.idx] = mm;
  c.idx = (c.idx + 1) % 3;
  float out = median3(c.hist[0], c.hist[1], c.hist[2]);   // kills single-sample spikes
  return out < 0 ? -1 : (int16_t)(out + 0.5f);
}

// Non-blocking: fire a slot, collect its echoes on the next slot, no busy-waiting.
static void usTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  int8_t prevSlot = -1;
  uint8_t slot = 0;
  for (;;) {
    if (prevSlot >= 0) {
      int16_t r[4]; bool have[4] = {false, false, false, false};
      for (int i = 0; i < 4; i++) {
        if (US_SLOT[i] == prevSlot) {
          r[i] = usFinalize(us[i]);
          have[i] = true;
          usN[i]++;
          if (r[i] >= 0) usOk[i]++;
        }
      }
      xSemaphoreTake(stateMtx, portMAX_DELAY);
      for (int i = 0; i < 4; i++) if (have[i]) S.us_mm[i] = r[i];
      xSemaphoreGive(stateMtx);
    }
    for (int i = 0; i < 4; i++) {
      if (US_SLOT[i] == slot) {
        us[i].gotRise = us[i].gotFall = false;
        digitalWrite(us[i].trig, HIGH);
      }
    }
    delayMicroseconds(10);
    for (int i = 0; i < 4; i++) if (US_SLOT[i] == slot) digitalWrite(us[i].trig, LOW);

    prevSlot = slot;
    slot ^= 1;
    vTaskDelayUntil(&lastWake, US_SLOT_TICKS);
  }
}

// ============================== Fast task (core 1, 200 Hz) ==================
static void fastTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastStart = 0;
  for (;;) {
    const uint32_t t0 = micros();
    if (lastStart && (t0 - lastStart) > FAST_PERIOD_US + 1500) fastLate++;
    lastStart = t0;

    updateEstop();
    if (!mainRelayCut) digitalWrite(PIN_MAIN_RELAY, HIGH);
    gyroRead();
    powerMonitor();
    estopStatusStep();
    smartSwitchStep();

    uint8_t ir = 0;
    for (size_t i = 0; i < sizeof(IR_PINS) && i < 8; i++)
      if (digitalRead(IR_PINS[i])) ir |= (1 << i);

    xSemaphoreTake(stateMtx, portMAX_DELAY);
    S.yaw_rad  = yaw2;
    S.gyro_ok  = gyroCalibrated;
    S.batt_pct = battPct;
    S.volts    = v24Filtered * 6.25f;
    S.amps     = batCurrentFiltered;
    S.charging = charging;
    S.estop    = estopLevel;
    S.cliffL   = (digitalRead(PIN_CLIFF_L) == CLIFF_ACTIVE_LEVEL);
    S.cliffR   = (digitalRead(PIN_CLIFF_R) == CLIFF_ACTIVE_LEVEL);
    S.ir_bits  = ir;
    xSemaphoreGive(stateMtx);

    fastCount++;
    const uint32_t busy = micros() - t0;
    if (busy > fastMaxUs) fastMaxUs = busy;
    vTaskDelayUntil(&lastWake, FAST_PERIOD_TICKS);
  }
}

// ============================== ESP-NOW (WCD) ===============================
static void queueWcd(const uint8_t *mac, const uint8_t *data, int len) {
  WcdMsg m;
  memcpy(m.mac, mac, 6);
  m.len = (len > WCD_MAX_PAYLOAD) ? WCD_MAX_PAYLOAD : len;
  memcpy(m.data, data, m.len);
  xQueueSend(wcdQ, &m, 0);   // callback stays short: just hand over to the comms task
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
static void onWcdRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  queueWcd(info->src_addr, data, len);
}
#else
static void onWcdRecv(const uint8_t *mac, const uint8_t *data, int len) {
  queueWcd(mac, data, len);
}
#endif

static bool espNowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onWcdRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, WCD_MAC, 6);
  peer.channel = 0;            // current channel (1 by default): the WCD must use the same
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// ============================== LEDs (core 0) ===============================
static void renderTray(CRGB *px, const TrayCmd &c, uint32_t ms) {
  const CRGB col(c.r, c.g, c.b);
  switch (c.pattern) {
    case 1: fill_solid(px, LEDS_PER_TRAY, col); break;
    case 2: fill_solid(px, LEDS_PER_TRAY, ((ms / 500) % 2 == 0) ? col : CRGB::Black); break;
    case 3: { CRGB b = col; b.nscale8_video(beatsin8(30, 10, 255)); fill_solid(px, LEDS_PER_TRAY, b); } break;
    case 4: {
      fill_solid(px, LEDS_PER_TRAY, CRGB::Black);
      const uint16_t pos = (ms / 20) % LEDS_PER_TRAY;
      for (int k = 0; k < 6; k++) px[(pos + k) % LEDS_PER_TRAY] = col;
    } break;
    case 5: fill_rainbow(px, LEDS_PER_TRAY, (ms / 20) & 0xFF, 3); break;
    default: fill_solid(px, LEDS_PER_TRAY, CRGB::Black); break;
  }
}

static void ledTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    TrayCmd c[3];
    bool dirty;
    portENTER_CRITICAL(&trayMux);
    memcpy(c, tray, sizeof(c));
    dirty = trayDirty;
    trayDirty = false;
    portEXIT_CRITICAL(&trayMux);

    bool animated = false;
    const uint32_t ms = millis();
    for (int t = 0; t < 3; t++) {
      if (c[t].pattern >= 2) animated = true;
      if (dirty || c[t].pattern >= 2) renderTray(leds[t], c[t], ms);
    }
    if (dirty || animated) FastLED.show();   // static patterns are only pushed when they change
    vTaskDelayUntil(&lastWake, 20);          // 50 fps
  }
}

// ============================== Comms (core 0) ==============================
static void txLine(const char *s, size_t n) {
  if (Serial.availableForWrite() >= (int)n) Serial.write((const uint8_t *)s, n);
  else txDrops++;                            // never block: drop instead of stalling
}

static void handleLine(char *line) {
  cmdCount++;
  if (line[0] == 'l' && line[1] == 'e' && line[2] == 'd') {
    int t, p, r, g, b;
    if (sscanf(line, "led,%d,%d,%d,%d,%d", &t, &p, &r, &g, &b) == 5 &&
        t >= 0 && t <= 3 && p >= 0 && p <= 5) {
      TrayCmd c = {(uint8_t)p, (uint8_t)constrain(r, 0, 255), (uint8_t)constrain(g, 0, 255),
                   (uint8_t)constrain(b, 0, 255)};
      portENTER_CRITICAL(&trayMux);
      for (int i = 0; i < 3; i++) if (t == 0 || t == i + 1) tray[i] = c;
      trayDirty = true;
      portEXIT_CRITICAL(&trayMux);
      txLine("ack,led\n", 8);
    } else {
      txLine("# bad led cmd\n", 14);
    }
  } else if (line[0] == 'w' && line[1] == ',') {
    const char *txt = line + 2;
    size_t n = strlen(txt);
    if (n > WCD_MAX_PAYLOAD) n = WCD_MAX_PAYLOAD;
    esp_now_send(WCD_MAC, (const uint8_t *)txt, n);
  } else if (line[0] == 'r' && (line[1] == '\0')) {
    yaw2 = 0.0f;
    txLine("ack,r\n", 6);
  } else {
    txLine("# unknown cmd\n", 14);
  }
}

static void pollSerialIn() {
  static char buf[256];
  static size_t n = 0;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n > 0) { buf[n] = '\0'; handleLine(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    } else {
      n = 0;                                 // line too long: discard
    }
  }
}

static void forwardWcd(const WcdMsg &m) {
  char out[512];
  int k = snprintf(out, sizeof(out), "w,%02X:%02X:%02X:%02X:%02X:%02X,",
                   m.mac[0], m.mac[1], m.mac[2], m.mac[3], m.mac[4], m.mac[5]);
  bool printable = true;
  for (int i = 0; i < m.len; i++) if (m.data[i] < 32 || m.data[i] > 126 || m.data[i] == ',') printable = false;
  if (printable) {
    for (int i = 0; i < m.len; i++) out[k++] = (char)m.data[i];
  } else {
    k += snprintf(out + k, sizeof(out) - k, "0x");
    for (int i = 0; i < m.len; i++) k += snprintf(out + k, sizeof(out) - k, "%02X", m.data[i]);
  }
  out[k++] = '\n';
  txLine(out, k);
  wcdRx++;
}

static void printStats() {
  static uint32_t lastMs = 0, lastFast = 0, lastTx = 0;
  static uint32_t lastOk[4] = {0}, lastN[4] = {0};
  const uint32_t now = millis();
  if (now - lastMs < STATS_PERIOD_MS) return;
  const float sec = (now - lastMs) / 1000.0f;
  lastMs = now;

  const uint32_t f = fastCount, t = txCount;
  char out[400];
  int k = snprintf(out, sizeof(out),
                   "# STATS fast=%.1fHz max_us=%lu late=%lu | tx=%.1fHz drops=%lu cmds=%lu wcd_rx=%lu | us_ok=",
                   (f - lastFast) / sec, (unsigned long)fastMaxUs, (unsigned long)fastLate,
                   (t - lastTx) / sec, (unsigned long)txDrops, (unsigned long)cmdCount,
                   (unsigned long)wcdRx);
  for (int i = 0; i < 4; i++) {
    const uint32_t ok = usOk[i], n = usN[i];
    k += snprintf(out + k, sizeof(out) - k, "%lu/%lu%s", (unsigned long)(ok - lastOk[i]),
                  (unsigned long)(n - lastN[i]), i < 3 ? "," : "\n");
    lastOk[i] = ok; lastN[i] = n;
  }
  lastFast = f; lastTx = t;
  fastMaxUs = 0;
  txLine(out, k);
}

static void commsTask(void *) {
  const TickType_t period = pdMS_TO_TICKS(1000 / SENSOR_RATE_HZ);
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t seq = 0;
  for (;;) {
    StatusMsg sm;
    while (xQueueReceive(statusQ, &sm, 0) == pdTRUE) {
      char line[48];
      const int n = snprintf(line, sizeof(line), "%s\n", sm.s);
      txLine(line, n);
    }
    WcdMsg wm;
    while (xQueueReceive(wcdQ, &wm, 0) == pdTRUE) forwardWcd(wm);

    pollSerialIn();

    SensorState c;
    xSemaphoreTake(stateMtx, portMAX_DELAY);
    c = S;
    xSemaphoreGive(stateMtx);

    uint32_t yawScaled;
    if (!c.gyro_ok || isnan(c.yaw_rad)) yawScaled = 44444;            // error flag, as on the Teensy
    else yawScaled = (uint32_t)((TO_DEG(c.yaw_rad) + 180.0f) * 100.0f);

    char pkt[160];
    const int n = snprintf(pkt, sizeof(pkt),
                           "d,%lu,%lu,%lu,%.1f,%d,%d,%.2f,%.2f,%d,%d,%d,%d,%u,%u,%u\n",
                           (unsigned long)seq++, (unsigned long)millis(), (unsigned long)yawScaled,
                           c.batt_pct, c.estop, c.charging ? 1 : 0, c.volts, c.amps,
                           c.us_mm[0], c.us_mm[1], c.us_mm[2], c.us_mm[3],
                           c.cliffL, c.cliffR, c.ir_bits);
    txLine(pkt, n);
    txCount++;

    if (STATS_ENABLED) printStats();
    vTaskDelayUntil(&lastWake, period);
  }
}

// ============================== Setup =======================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  Serial.println("# booting ESP32 sensor hub");

  pinMode(PIN_MAIN_RELAY, OUTPUT);
  digitalWrite(PIN_MAIN_RELAY, HIGH);
  pinMode(PIN_MAIN_SW_FB, INPUT_PULLUP);
  pinMode(PIN_ESTOP_FB, INPUT_PULLUP);
  estopLevel = digitalRead(PIN_ESTOP_FB);   // start from the real state, no debounce wait at boot
  estopReleasedMs = millis();
  pinMode(PIN_CLIFF_L, INPUT);
  pinMode(PIN_CLIFF_R, INPUT);
  for (size_t i = 0; i < sizeof(IR_PINS); i++) pinMode(IR_PINS[i], INPUT);

  stateMtx = xSemaphoreCreateMutex();
  statusQ = xQueueCreate(16, sizeof(StatusMsg));
  wcdQ = xQueueCreate(8, sizeof(WcdMsg));
  memset(&S, 0, sizeof(S));
  S.estop = estopLevel;
  for (int i = 0; i < 4; i++) S.us_mm[i] = -1;

  if (!gyroInit()) Serial.println("# WARNING: BMI088 not found, yaw will report 44444");

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_ADC_CS);
  pinMode(PIN_ADC_CS, OUTPUT);
  digitalWrite(PIN_ADC_CS, HIGH);
  for (int i = 0; i < 250; i++) {           // prime the filters so a charger is detected right at boot
    powerMonitor();
    delayMicroseconds(500);
  }
  Serial.printf("# boot: charging=%d batt=%.1f%%\n", charging ? 1 : 0, battPct);

  for (int i = 0; i < 4; i++) {
    us[i].trig = US_TRIG[i];
    us[i].echo = US_ECHO[i];
    for (int h = 0; h < 3; h++) us[i].hist[h] = -1.0f;
    pinMode(us[i].trig, OUTPUT);
    digitalWrite(us[i].trig, LOW);
    pinMode(us[i].echo, INPUT);
    // setup() runs on core 1, so these edge interrupts are serviced on core 1 too
    attachInterruptArg(digitalPinToInterrupt(us[i].echo), usIsr, &us[i], CHANGE);
  }

  FastLED.addLeds<WS2812B, PIN_LED1, GRB>(leds[0], LEDS_PER_TRAY);
  FastLED.addLeds<WS2812B, PIN_LED2, GRB>(leds[1], LEDS_PER_TRAY);
  FastLED.addLeds<WS2812B, PIN_LED3, GRB>(leds[2], LEDS_PER_TRAY);
  FastLED.setBrightness(LED_BRIGHTNESS);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, LED_MAX_MA);
  FastLED.clear(true);

  if (!espNowInit()) Serial.println("# WARNING: ESP-NOW init failed (WCD link disabled)");

  // core 1: timing-critical sensing   |   core 0: comms, ESP-NOW, LEDs
  xTaskCreatePinnedToCore(fastTask,  "fast",  6144, nullptr, 6, nullptr, 1);
  xTaskCreatePinnedToCore(usTask,    "us",    3072, nullptr, 5, nullptr, 1);
  xTaskCreatePinnedToCore(commsTask, "comms", 8192, nullptr, 4, nullptr, 0);
  xTaskCreatePinnedToCore(ledTask,   "led",   4096, nullptr, 3, nullptr, 0);
}

void loop() {
  vTaskDelete(nullptr);   // everything runs in the tasks above
}

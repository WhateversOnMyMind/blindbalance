/*
 * Arm-swing tracker + ESP-NOW balance feedback
 * ONE IMU per MCU (ESP32-S3-MINI-1 + one ICM-42688-P) — Composite Board rev C
 *
 * Two independent boards. Flash this SAME sketch to BOTH; change only the
 * ARM toggle below. Each board:
 *   - tracks its own arm's swings,
 *   - posts its own {arm, count, distance} to Google Sheets every 10 s,
 *   - broadcasts its recent swing window to the other board over ESP-NOW,
 *   - compares the two arms and buzzes ITS OWN motor when its arm lags.
 *
 * Pins from Composite Board rev C:
 *   SPI   MISO=IO8  MOSI=IO9  SCLK=IO10  CS=IO11        (IMU U3)
 *   MOTOR net "IN" = IO14                               (U4 vibe driver input)
 *   NOTE: U4 is crossed-out (DNP) on rev C — confirm the motor/driver is
 *         actually populated before expecting buzzes.
 *
 * ESP-NOW note: both boards join the same AP, so they sit on the same WiFi
 * channel automatically and ESP-NOW + HTTP coexist. Init ESP-NOW AFTER WiFi.
 */

// ─────────── pick this board's arm ───────────
//#define ARM_IS_LEFT          // <-- comment out on the RIGHT board
// ─────────────────────────────────────────────

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <time.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_now.h>

#ifdef ARM_IS_LEFT
  static const char ARM_CHAR = 'L';
#else
  static const char ARM_CHAR = 'R';
#endif
static const bool I_AM_LEFT = (ARM_CHAR == 'L');

/* ── WiFi ── */
const char* ssid      = "Aaa";
const char* password  = "yoyoyoyo";
const char* scriptURL = "https://script.google.com/macros/s/AKfycbynO6PeezcsT10WQaf6S5EsOnATbqMygyd8eIJhjy_2cd5CxTcDteisPzt5JTQeayjBxg/exec";

/* ── SPI pin map (single IMU) ── */
static const int PIN_MISO = 8;
static const int PIN_MOSI = 9;
static const int PIN_SCLK = 10;
static const int PIN_CS   = 11;

/* ── this board's single vibration motor (net "IN") ── */
#define PIN_VIBE  14

/* ── ICM-42688-P registers ── */
#define REG_DEVICE_CONFIG   0x11
#define REG_TEMP_DATA1      0x1D
#define REG_PWR_MGMT0       0x4E
#define REG_GYRO_CONFIG0    0x4F
#define REG_ACCEL_CONFIG0   0x50
#define REG_WHO_AM_I        0x75
#define WHO_AM_I_EXPECTED   0x47

static const float GYRO_LSB_PER_DPS = 16.4f;    // ±2000 dps
static const float ACCEL_LSB_PER_G  = 2048.0f;  // ±16 g
static const float DEG2RAD = 0.0174532925f;
static const float G2MS2   = 9.80665f;

static uint32_t spiHz = 8000000;
SPIClass spi(FSPI);

/* ── sampling & calibration ── */
#define SAMPLE_RATE_HZ  250
#define CALIB_TIME_MS   2000

/* ── swing thresholds ── */
#define GYRO_THRESH_FWD      0.4f
#define GYRO_THRESH_BACK     0.4f
#define RADIUS_DEFAULT       0.30f
#define ACCEL_MAG_THRESH_DELTA 2.0f
#define SWING_MAX_MS         1200
#define WY_IDLE              0.12f
#define END_ZERO_CROSS_MS    180
#define BIAS_ADAPT_ALPHA     0.0008f

/* ── window timing ── */
#define SHEETS_WINDOW_MS  10000     // Google Sheets upload cadence (unchanged)
#define FB_WINDOW_MS      2000      // haptic-feedback comparison window
#define FB_PEER_TIMEOUT   3000      // peer silent longer than this -> motor off

/* ── feedback shaping ──
 * delta = 0.5 - myShare  (>0 means THIS arm is lagging).
 * Buzz starts past the dead zone and ramps to full at maximum imbalance.
 * Raise FB_DEAD_ZONE to ~0.20 to reproduce the old 0.30 hard gate. */
#define FB_DEAD_ZONE      0.055f

struct Sample { float ax, ay, az, gx, gy, gz, tempC; };

Sample g_lastSample;   // most recent IMU reading, shared so RawLog.ino doesn't need its own SPI read

/* ── this board's arm state ── */
float    g_bias = 0.0f;             // rad/s
float    g_radius = RADIUS_DEFAULT;
bool     g_inSwing = false;
int      g_dir = 0;
float    g_angle = 0.0f;
float    g_startAngle = 0.0f;
uint32_t g_swingCount = 0;          // Sheets window
float    g_swingDistanceSum = 0.0f; // Sheets window (m)
uint32_t g_fbCount = 0;             // feedback window
float    g_fbDist  = 0.0f;          // feedback window (m)
uint32_t g_lastSwingTime = 0;
volatile bool g_motorEnabled = true;   // LEFT sets via /toggle; RIGHT adopts from peer packets
uint32_t g_swingStartMs = 0;

uint64_t lastMicros = 0;
unsigned long lastSendTime = 0;
unsigned long lastFbTime   = 0;

/* ── ESP-NOW peer exchange ── */
typedef struct __attribute__((packed)) {
  char     arm;          // 'L' or 'R'
  uint32_t count;        // peer's last feedback-window swing count
  float    distance;     // peer's last feedback-window arc length (m)
  bool     motorEnabled; // sender's motor mode (only 'L' is authoritative)
} PeerMsg;

static uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
volatile uint32_t peerCount = 0;
volatile float    peerDist  = 0.0f;
volatile uint32_t peerRxMs  = 0;
volatile char     peerArm   = '?';

/* ── SPI transport ── */
uint8_t regRead(uint8_t reg) {
  spi.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_CS, LOW);
  spi.transfer(reg | 0x80);
  uint8_t v = spi.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  spi.endTransaction();
  return v;
}
void regReadBurst(uint8_t reg, uint8_t* buf, size_t n) {
  spi.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_CS, LOW);
  spi.transfer(reg | 0x80);
  for (size_t i = 0; i < n; i++) buf[i] = spi.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  spi.endTransaction();
}
void regWrite(uint8_t reg, uint8_t val) {
  spi.beginTransaction(SPISettings(spiHz, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_CS, LOW);
  spi.transfer(reg & 0x7F);
  spi.transfer(val);
  digitalWrite(PIN_CS, HIGH);
  spi.endTransaction();
  delayMicroseconds(200);
}

Sample readSample() {
  uint8_t r[14];
  regReadBurst(REG_TEMP_DATA1, r, 14);
  auto i16 = [](uint8_t hi, uint8_t lo){ return (int16_t)((hi << 8) | lo); };
  Sample s;
  s.tempC = i16(r[0],  r[1])  / 132.48f + 25.0f;
  s.ax    = i16(r[2],  r[3])  / ACCEL_LSB_PER_G;
  s.ay    = i16(r[4],  r[5])  / ACCEL_LSB_PER_G;
  s.az    = i16(r[6],  r[7])  / ACCEL_LSB_PER_G;
  s.gx    = i16(r[8],  r[9])  / GYRO_LSB_PER_DPS;
  s.gy    = i16(r[10], r[11]) / GYRO_LSB_PER_DPS;
  s.gz    = i16(r[12], r[13]) / GYRO_LSB_PER_DPS;
  return s;
}

bool initIMU() {
  regWrite(REG_DEVICE_CONFIG, 0x01); delay(5);
  regWrite(REG_GYRO_CONFIG0,  0x06);
  regWrite(REG_ACCEL_CONFIG0, 0x06);
  regWrite(REG_PWR_MGMT0,     0x0F); delay(100);
  uint8_t who = regRead(REG_WHO_AM_I);
  Serial.printf("IMU %c  WHO_AM_I = 0x%02X\n", ARM_CHAR, who);
  return who == WHO_AM_I_EXPECTED;
}

void calibrateGyro() {
  Serial.print(F("Calibrating ")); Serial.println(ARM_CHAR);
  delay(400);
  uint32_t t0 = millis();
  float sum = 0; uint32_t n = 0;
  while (millis() - t0 < CALIB_TIME_MS) {
    sum += readSample().gy * DEG2RAD;   // <-- swing axis (change if not gy)
    ++n; delay(4);
  }
  g_bias = sum / n;
  g_angle = 0; g_dir = 0; g_inSwing = false; g_swingCount = 0;
  Serial.print(ARM_CHAR); Serial.print(F(" bias = "));
  Serial.println(g_bias, 6);
}

void printSwing(float endAngle) {
  uint32_t now = millis();
  if (now - g_lastSwingTime < 400) {
    // debounce quick repeats, but still end/reset below
  } else {
    float dTheta = endAngle - g_startAngle;
    ++g_swingCount;
    ++g_fbCount;
    float arcLen = fabsf(dTheta) * g_radius;
    g_swingDistanceSum += arcLen;
    g_fbDist           += arcLen;
    Serial.print(F("Swing ")); Serial.print(ARM_CHAR);
    Serial.print(F(" #")); Serial.print(g_swingCount);
    Serial.print(F(" | Angle: ")); Serial.print(fabsf(dTheta) * 57.29578f, 1);
    Serial.print(F(" deg | Length: ")); Serial.print(arcLen * 100.0f, 1);
    Serial.println(F(" cm"));
    g_lastSwingTime = now;
  }
  g_inSwing = false;
  g_dir = 0;
}

void updateArm(float dt) {
  Sample s = readSample();
  g_lastSample = s;

  // ── SWING AXIS: original used Y. If not Y on this mounting, change s.gy
  //    -> s.gx or s.gz here AND in calibrateGyro(). ──
  float wy_raw = s.gy * DEG2RAD;               // rad/s
  float wy = wy_raw - g_bias;
  g_angle += wy * dt;

  float ax = s.ax * G2MS2, ay = s.ay * G2MS2, az = s.az * G2MS2;
  float aMag = sqrtf(ax*ax + ay*ay + az*az);   // m/s^2
  bool accelTrigger = fabsf(aMag - 9.8f) > ACCEL_MAG_THRESH_DELTA;

  if (!g_inSwing) {
    if (accelTrigger) {
      if (wy > GYRO_THRESH_FWD) {
        g_inSwing = true; g_dir = +1; g_startAngle = g_angle; g_swingStartMs = millis();
      } else if (wy < -GYRO_THRESH_BACK) {
        g_inSwing = true; g_dir = -1; g_startAngle = g_angle; g_swingStartMs = millis();
      }
    }
  } else {
    bool crossed = (g_dir == +1 && wy < -GYRO_THRESH_BACK) ||
                   (g_dir == -1 && wy >  GYRO_THRESH_FWD);
    uint32_t elapsed = millis() - g_swingStartMs;
    bool zeroLike = (fabsf(wy) < WY_IDLE) && (elapsed > END_ZERO_CROSS_MS);
    bool timeout  = elapsed > SWING_MAX_MS;
    if (crossed || zeroLike || timeout) printSwing(g_angle);
  }

  if (fabsf(wy) < WY_IDLE)
    g_bias = (1.0f - BIAS_ADAPT_ALPHA) * g_bias + BIAS_ADAPT_ALPHA * wy_raw;
}

/* ── post THIS arm's 10 s window ── */
void sendToGoogleSheets() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(scriptURL);
  http.addHeader("Content-Type", "application/json");

  float cm = g_swingDistanceSum * 100.0f;
  String json = "{";
  json += "\"arm\": \""   + String(ARM_CHAR) + "\",";
  json += "\"count\": "    + String(g_swingCount) + ",";
  json += "\"distance\": " + String(cm, 2);
  json += "}";

  Serial.println("Sending JSON: " + json);
  int code = http.POST(json);
  Serial.print("HTTP Response code: "); Serial.println(code);
  http.end();

  g_swingCount = 0;
  g_swingDistanceSum = 0;
}

/* ── ESP-NOW receive: store the OTHER arm's latest window ── */
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t* /*info*/, const uint8_t* data, int len) {
#else
void onRecv(const uint8_t* /*mac*/, const uint8_t* data, int len) {
#endif
  if (len != sizeof(PeerMsg)) return;
  PeerMsg m; memcpy(&m, data, sizeof(m));
  if (m.arm == ARM_CHAR) return;         // ignore my own / same-arm packets
  peerArm   = m.arm;
  peerCount = m.count;
  peerDist  = m.distance;
  peerRxMs  = millis();
  if (!I_AM_LEFT && m.arm == 'L') g_motorEnabled = m.motorEnabled;
}

void initEspNow() {
  WiFi.setSleep(false);                  // keep radio awake for reliable ESP-NOW
  if (esp_now_init() != ESP_OK) { Serial.println(F("ESP-NOW init failed")); return; }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, BCAST, 6);
  p.channel = 0;                         // 0 = use current STA channel
  p.ifidx   = WIFI_IF_STA;
  p.encrypt = false;
  if (esp_now_add_peer(&p) != ESP_OK) Serial.println(F("ESP-NOW add peer failed"));
  Serial.printf("ESP-NOW ready on channel %d\n", WiFi.channel());
}

/* ── check if the peer's data is fresh (used for cross-tab peer-state tracking) ── */
bool peerIsFresh() {
  return (peerArm != '?') && (millis() - peerRxMs < FB_PEER_TIMEOUT);
}

/* ── run every FB_WINDOW_MS: swap windows, compute balance, drive own motor ── */
void feedbackTick() {
  uint32_t myC = g_fbCount;  float myD = g_fbDist;
  g_fbCount = 0; g_fbDist = 0;

  // broadcast my just-finished window
  PeerMsg m; m.arm = ARM_CHAR; m.count = myC; m.distance = myD; m.motorEnabled = g_motorEnabled;
  esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));

  // need a live peer to compute balance
  bool peerFresh = peerIsFresh();
  if (!peerFresh) { analogWrite(PIN_VIBE, 0); return; }

  uint32_t pC = peerCount; float pD = peerDist;

  uint32_t lc, rc; float ld, rd;
  if (I_AM_LEFT) { lc = myC; ld = myD; rc = pC; rd = pD; }
  else           { rc = myC; rd = myD; lc = pC; ld = pD; }

  if ((lc + rc) == 0 || (ld + rd) <= 0.0f) { analogWrite(PIN_VIBE, 0); return; }

  float countFracL = (float)lc / (float)(lc + rc);
  float distFracL  = ld / (ld + rd);
  float leftShare  = (countFracL + distFracL) / 2.0f;
  float myShare    = I_AM_LEFT ? leftShare : (1.0f - leftShare);

  float delta = 0.5f - myShare;              // >0 => my arm is lagging
  if (delta <= FB_DEAD_ZONE) { analogWrite(PIN_VIBE, 0); return; }  // balanced or ahead

  float intensity = (delta - FB_DEAD_ZONE) / (0.5f - FB_DEAD_ZONE);
  if (intensity < 0) intensity = 0;
  if (intensity > 1) intensity = 1;
  int pwm = (int)(255.0f * intensity);

  Serial.printf("[%c] leftShare=%.2f myShare=%.2f -> buzz %d\n",
                ARM_CHAR, leftShare, myShare, pwm);
  analogWrite(PIN_VIBE, g_motorEnabled ? pwm : 0);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
  spi.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  delay(50);

  if (!initIMU()) { Serial.println(F("IMU not found!")); while (1) delay(10); }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi connected!");

  configTime(0, 0, "pool.ntp.org", "time.nist.gov");   // UTC epoch time for event logging
  Serial.print(F("Syncing time"));
  uint32_t ntpStart = millis();
  while (time(nullptr) < 1000000000UL && millis() - ntpStart < 5000) {
    delay(100);
    Serial.print(".");
  }
  Serial.println();

  initEspNow();                  // after WiFi so we inherit the AP's channel
  webUiBegin();                  // start local web server + mDNS
  rawLogBegin();                 // mount LittleFS for raw-log capture sessions

  calibrateGyro();               // keep the arm STILL here

  pinMode(PIN_VIBE, OUTPUT);
  analogWrite(PIN_VIBE, 0);

  lastMicros = micros();
  lastSendTime = millis();
  lastFbTime   = millis();
  g_lastSwingTime = 0; g_swingStartMs = 0;
}

void loop() {
  uint64_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  updateArm(dt);
  webUiHandleClient();           // service any pending HTTP requests
  rawLogTick();                  // capture a raw sample row if a log session is active

  if (millis() - lastFbTime >= FB_WINDOW_MS) {
    feedbackTick();
    lastFbTime = millis();
  }
  if (millis() - lastSendTime >= SHEETS_WINDOW_MS) {
    sendToGoogleSheets();
    lastSendTime = millis();
  }

  delayMicroseconds(1000000 / SAMPLE_RATE_HZ - 200);
}

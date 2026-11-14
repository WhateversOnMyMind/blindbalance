/* ============================================================
   Arm-swing tracker — SINGLE device
   Seeed Studio XIAO ESP32S3 + one Adafruit ICM-20948 (PID 4554)

   One device per arm. Flash the SAME sketch to both, just flip
   the SIDE toggle below for each one. No motors / no feedback —
   the device only counts swings and uploads to Google Sheets.

   Libraries (Arduino Library Manager):
     - "Adafruit ICM20X"  (provides Adafruit_ICM20948.h)
     - "Adafruit Unified Sensor"
     - "Adafruit BusIO"
   Board: "XIAO_ESP32S3" (esp32 Arduino core 3.x)

   Wiring: SDA = GPIO5 (D4), SCL = GPIO6 (D5).
   I2C address 0x68 (confirmed by scan on this board). Each arm
   has its own microcontroller/bus, so no address juggling needed.
   ============================================================ */

// ===== SELECT WHICH ARM THIS DEVICE IS =====
#define SIDE_LEFT          // <-- LEFT device: leave as-is
// #define SIDE_RIGHT      // <-- RIGHT device: comment the line above, uncomment this
// ============================================

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#if defined(SIDE_LEFT) && defined(SIDE_RIGHT)
  #error "Pick only ONE of SIDE_LEFT / SIDE_RIGHT"
#elif defined(SIDE_LEFT)
  #define SIDE_LABEL  "L"
  #define KEY_COUNT   "leftCount"
  #define KEY_DIST    "leftDistance"
#elif defined(SIDE_RIGHT)
  #define SIDE_LABEL  "R"
  #define KEY_COUNT   "rightCount"
  #define KEY_DIST    "rightDistance"
#else
  #error "Define SIDE_LEFT or SIDE_RIGHT"
#endif

/* ── WiFi ── */
const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* scriptURL = "YOUR_APPS_SCRIPT_EXEC_URL";

/* ── XIAO ESP32S3 I2C pins ── */
#define SDA_PIN              5     // D4
#define SCL_PIN              6     // D5

/* ── I2C address (confirmed by scan: WHO_AM_I = 0xEA) ── */
#define ICM_ADDR             0x69

/* ── sampling & calibration ── */
#define SAMPLE_RATE_HZ       250
#define CALIB_TIME_MS        2000

/* ── thresholds ── */
#define GYRO_THRESH_FWD      0.4f
#define GYRO_THRESH_BACK     0.4f
#define MIN_DELTA_THETA      0.2f
#define RADIUS_DEFAULT       0.30f
#define ACCEL_MAG_THRESH_DELTA 2.0f

/* ── swing robustness ── */
#define SWING_MAX_MS         1200   // max swing duration before auto-end
#define WY_IDLE              0.12f  // "effectively stopped" angular speed
#define END_ZERO_CROSS_MS    180    // wait at least this long before zero-end
#define BIAS_ADAPT_ALPHA     0.0008f  // slow drift adapt only when idle

Adafruit_ICM20948 icm;

struct Arm {
  float bias;
  float radius;
  bool inSwing;
  int dir;
  float angle;
  float startAngle;
  uint32_t swingCount;
  float swingDistanceSum;
  const char* label;
  uint32_t lastSwingTime;
  uint32_t swingStartMs;
};

Arm arm = { 0.0f, RADIUS_DEFAULT, false, 0, 0.0f, 0.0f, 0, 0.0f, SIDE_LABEL, 0, 0 };

uint64_t lastMicros = 0;
unsigned long lastSendTime = 0;

/* Match the old MPU6050 settings. getEvent() returns gyro in rad/s
   and accel in m/s^2, so every threshold below is unchanged. */
void configIMU(Adafruit_ICM20948 &dev) {
  dev.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);   // was MPU6050_RANGE_500_DEG
  dev.setAccelRange(ICM20948_ACCEL_RANGE_8_G);     // headroom for swing spikes
  dev.setGyroRateDivisor(3);   // ~275 Hz
  dev.setAccelRateDivisor(3);  // ~281 Hz
}

void calibrateGyro(Arm &a_) {
  Serial.print(F("Calibrating ")); Serial.println(a_.label);
  delay(400);
  uint32_t t0 = millis();
  float sum = 0; uint32_t n = 0;
  sensors_event_t a, g, t, m;
  while (millis() - t0 < CALIB_TIME_MS) {
    icm.getEvent(&a, &g, &t, &m);
    sum += g.gyro.y; ++n; delay(4);
  }
  a_.bias = sum / n;
  a_.angle = 0; a_.dir = 0; a_.inSwing = false; a_.swingCount = 0;
  Serial.print(a_.label); Serial.print(F(" bias = "));
  Serial.println(a_.bias, 6);
}

void printSwing(Arm &a_, float endAngle) {
  uint32_t now = millis();
  if (now - a_.lastSwingTime < 400) {
    // Debounce: don't double-count quick repeats, but still reset state below.
  } else {
    float dTheta = endAngle - a_.startAngle;
    ++a_.swingCount;
    float arcLen = fabsf(dTheta) * a_.radius;
    a_.swingDistanceSum += arcLen;
    Serial.print(F("Swing ")); Serial.print(a_.label);
    Serial.print(F(" #")); Serial.print(a_.swingCount);
    Serial.print(F(" │ Angle: ")); Serial.print(fabsf(dTheta) * 57.29578f, 1);
    Serial.print(F(" ° │ Length: ")); Serial.print(arcLen * 100.0f, 1);
    Serial.println(F(" cm"));
    a_.lastSwingTime = now;
  }
  a_.inSwing = false;
  a_.dir = 0;
}

void updateArm(Arm &a_, float dt) {
  sensors_event_t a, g, t, m;
  icm.getEvent(&a, &g, &t, &m);

  float wy_raw = g.gyro.y;
  float wy = wy_raw - a_.bias;
  a_.angle += wy * dt;

  float aMag = sqrtf(a.acceleration.x * a.acceleration.x +
                     a.acceleration.y * a.acceleration.y +
                     a.acceleration.z * a.acceleration.z);
  bool accelTrigger = fabsf(aMag - 9.8f) > ACCEL_MAG_THRESH_DELTA;

  if (!a_.inSwing) {
    if (accelTrigger) {
      if (wy > GYRO_THRESH_FWD) {
        a_.inSwing = true; a_.dir = +1; a_.startAngle = a_.angle; a_.swingStartMs = millis();
      } else if (wy < -GYRO_THRESH_BACK) {
        a_.inSwing = true; a_.dir = -1; a_.startAngle = a_.angle; a_.swingStartMs = millis();
      }
    }
  } else {
    bool crossed = (a_.dir == +1 && wy < -GYRO_THRESH_BACK) ||
                   (a_.dir == -1 && wy >  GYRO_THRESH_FWD);
    uint32_t elapsed = millis() - a_.swingStartMs;
    bool zeroLike = (fabsf(wy) < WY_IDLE) && (elapsed > END_ZERO_CROSS_MS);
    bool timeout = elapsed > SWING_MAX_MS;
    if (crossed || zeroLike || timeout) {
      printSwing(a_, a_.angle);
    }
  }

  if (fabsf(wy) < WY_IDLE) {
    a_.bias = (1.0f - BIAS_ADAPT_ALPHA) * a_.bias + BIAS_ADAPT_ALPHA * wy_raw;
  }
}

void sendToGoogleSheets() {
  if (WiFi.status() != WL_CONNECTED) return;

  Serial.print("RSSI: ");      Serial.println(WiFi.RSSI());        // signal strength (dBm)
  Serial.print("Free heap: "); Serial.println(ESP.getFreeHeap());  // TLS needs RAM

  WiFiClientSecure client;
  client.setInsecure();
  client.setHandshakeTimeout(30);     // Google's TLS handshake can be slow

  HTTPClient http;
  http.begin(client, scriptURL);
  http.setConnectTimeout(10000);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.addHeader("Content-Type", "application/json");

  float cm = arm.swingDistanceSum * 100.0f;

  String json = "{";
  json += "\"" KEY_COUNT "\": " + String(arm.swingCount) + ",";
  json += "\"" KEY_DIST  "\": " + String(cm, 2);
  json += "}";

  Serial.println("Sending JSON: " + json);
  int code = http.POST(json);
  Serial.print("HTTP Response code: ");
  Serial.println(code);
  if (code <= 0) {
    Serial.print("Error: ");
    Serial.println(http.errorToString(code));
  }
  http.end();

  // Reset for next 10s window
  arm.swingCount = 0;
  arm.swingDistanceSum = 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  if (!icm.begin_I2C(ICM_ADDR, &Wire)) {
    Serial.println(F("ICM-20948 not found (0x68)!")); while (1) delay(10);
  }
  configIMU(icm);

  Serial.print(F("This device = arm ")); Serial.println(arm.label);
  calibrateGyro(arm);

  lastMicros = micros();
  lastSendTime = millis();
  arm.lastSwingTime = 0; arm.swingStartMs = 0;
}

void loop() {
  uint64_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  updateArm(arm, dt);

  if (millis() - lastSendTime >= 10000) {
    if (arm.swingCount > 0) {     // skip idle windows → no zero-flood
      sendToGoogleSheets();       // resets counters for the next window
    }
    lastSendTime = millis();
  }

  delayMicroseconds(1000000 / SAMPLE_RATE_HZ - 200);
}

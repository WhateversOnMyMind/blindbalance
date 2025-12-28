#include <Adafruit_MPU6050.h>

#include <Adafruit_Sensor.h>

#include <Wire.h>

#include <WiFi.h>

#include <HTTPClient.h>

/* ── WiFi ── */

const char* ssid     = "Aaa";

const char* password = "yoyoyoyo";

const char* scriptURL = "https://script.google.com/macros/s/AKfycbzBvJKEtq9sOr6JkIPRjNUf_4xROrjfYzzAtlElzlUsUg69tv45QoupNosSsXKZQs7l/exec";

/* ── pins ── */

#define SDA_PIN              21

#define SCL_PIN              22

#define PIN_VIBE_LEFT   16

#define PIN_VIBE_RIGHT  17

/* ── addresses ── */

#define ADDR_LEFT            0x68

#define ADDR_RIGHT           0x69

/* ── sampling & calibration ── */

#define SAMPLE_RATE_HZ       250

#define CALIB_TIME_MS        2000

/* ── thresholds ── */

#define GYRO_THRESH_FWD      0.4f

#define GYRO_THRESH_BACK     0.4f

#define MIN_DELTA_THETA      0.2f

#define RADIUS_DEFAULT       0.30f

#define ACCEL_MAG_THRESH_DELTA 2.0f

/* ── swing robustness (NEW) ── */

#define SWING_MAX_MS         1200   // max swing duration before auto-end

#define WY_IDLE              0.12f  // "effectively stopped" angular speed

#define END_ZERO_CROSS_MS    180    // wait at least this long before zero-end

#define BIAS_ADAPT_ALPHA     0.0008f  // slow drift adapt only when idle

Adafruit_MPU6050 mpuLeft;

Adafruit_MPU6050 mpuRight;

struct Arm {

  Adafruit_MPU6050* mpu;

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

  uint32_t swingStartMs;   // NEW: timestamp when swing started

};

Arm left = { &mpuLeft, 0.0f, RADIUS_DEFAULT, false, 0, 0.0f, 0.0f, 0, 0.0f, "L", 0, 0 };

Arm right = { &mpuRight, 0.0f, RADIUS_DEFAULT, false, 0, 0.0f, 0.0f, 0, 0.0f, "R", 0, 0 };

uint64_t lastMicros = 0;

unsigned long lastSendTime = 0;

void calibrateGyro(Arm &arm) {

  Serial.print(F("Calibrating ")); Serial.println(arm.label);

  delay(400);

  uint32_t t0 = millis();

  float sum = 0; uint32_t n = 0;

  sensors_event_t a, g, t;

  while (millis() - t0 < CALIB_TIME_MS) {

    arm.mpu->getEvent(&a, &g, &t);

    sum += g.gyro.y; ++n; delay(4);

  }

  arm.bias = sum / n;

  arm.angle = 0; arm.dir = 0; arm.inSwing = false; arm.swingCount = 0;

  Serial.print(arm.label); Serial.print(F(" bias = "));

  Serial.println(arm.bias, 6);

}

void printSwing(Arm &arm, float endAngle) {

  uint32_t now = millis();

  if (now - arm.lastSwingTime < 400) {

    // Debounce: don't double-count very quick repeats,

    // but DO end/reset the swing state below.

  } else {

    float dTheta = endAngle - arm.startAngle;

    ++arm.swingCount;

    float arcLen = fabsf(dTheta) * arm.radius;

    arm.swingDistanceSum += arcLen;

    Serial.print(F("Swing ")); Serial.print(arm.label);

    Serial.print(F(" #")); Serial.print(arm.swingCount);

    Serial.print(F(" │ Angle: ")); Serial.print(fabsf(dTheta) * 57.29578f, 1);

    Serial.print(F(" ° │ Length: ")); Serial.print(arcLen * 100.0f, 1);

    Serial.println(F(" cm"));

    arm.lastSwingTime = now;

  }

  // Always end/reset the state

  arm.inSwing = false;

  arm.dir = 0;

}

void updateArm(Arm &arm, float dt) {

  sensors_event_t a, g, t;

  arm.mpu->getEvent(&a, &g, &t);

  float wy_raw = g.gyro.y;

  float wy = wy_raw - arm.bias;

  arm.angle += wy * dt;

  // total acceleration magnitude

  float aMag = sqrtf(a.acceleration.x * a.acceleration.x +

                     a.acceleration.y * a.acceleration.y +

                     a.acceleration.z * a.acceleration.z);

  // deviation from gravity (~9.8 m/s^2)

  bool accelTrigger = fabsf(aMag - 9.8f) > ACCEL_MAG_THRESH_DELTA;

  if (!arm.inSwing) {

    if (accelTrigger) {

      if (wy > GYRO_THRESH_FWD) {

        arm.inSwing = true; arm.dir = +1; arm.startAngle = arm.angle; arm.swingStartMs = millis();

      } else if (wy < -GYRO_THRESH_BACK) {

        arm.inSwing = true; arm.dir = -1; arm.startAngle = arm.angle; arm.swingStartMs = millis();

      }

    }

  } else {

    // normal end: crossing the opposite threshold

    bool crossed = (arm.dir == +1 && wy < -GYRO_THRESH_BACK) ||

                   (arm.dir == -1 && wy >  GYRO_THRESH_FWD);

    uint32_t elapsed = millis() - arm.swingStartMs;

    // slow / nearly stopped + minimum duration → end

    bool zeroLike = (fabsf(wy) < WY_IDLE) && (elapsed > END_ZERO_CROSS_MS);

    // timeout → end regardless

    bool timeout = elapsed > SWING_MAX_MS;

    if (crossed || zeroLike || timeout) {

      printSwing(arm, arm.angle);

    }

  }

  // very low speed → gently adapt bias to fight long-term drift

  if (fabsf(wy) < WY_IDLE) {

    arm.bias = (1.0f - BIAS_ADAPT_ALPHA) * arm.bias + BIAS_ADAPT_ALPHA * wy_raw;

  }

}

/* ── DO NOT CHANGE: Google Sheets output kept EXACTLY as your original ── */

void sendToGoogleSheets() {

  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;

  http.begin(scriptURL);

  http.addHeader("Content-Type", "application/json");

  // cm and counts

  float left_cm = left.swingDistanceSum * 100.0f;

  float right_cm = right.swingDistanceSum * 100.0f;

  String json = "{";

  json += "\"leftCount\": " + String(left.swingCount) + ",";

  json += "\"leftDistance\": " + String(left_cm, 2) + ",";

  json += "\"rightCount\": " + String(right.swingCount) + ",";

  json += "\"rightDistance\": " + String(right_cm, 2);

  json += "}";

  Serial.println("Sending JSON: " + json);

  int code = http.POST(json);

  Serial.print("HTTP Response code: ");

  Serial.println(code);

  http.end();

  // Reset for next 10s window

  left.swingCount = 0;

  right.swingCount = 0;

  left.swingDistanceSum = 0;

  right.swingDistanceSum = 0;

}

void feedback() {

  // Snapshot (10s window, because sendToGoogleSheets() resets after each send)

  uint32_t lc = left.swingCount,  rc = right.swingCount;

  float    ld = left.swingDistanceSum, rd = right.swingDistanceSum;

  // Guards

  if ((lc + rc) == 0 || (ld + rd) == 0) {

    analogWrite(PIN_VIBE_LEFT, 0);

    analogWrite(PIN_VIBE_RIGHT, 0);

    return;

  }

  // Fractions for how much the LEFT contributed (0..1)

  float countFracL = float(lc) / float(lc + rc);

  float distFracL  =  ld       / (ld + rd);

  // Combine shares (0.5 = balanced)

  float leftShare = (countFracL + distFracL) / 2.0f;

  // Dead zone around perfect balance (±5.5%)

  const float DZ = 0.055f;

  // Distance from balance

  float delta = fabs(leftShare - 0.5f);        // 0..0.5

  if (delta <= DZ) {

    // Balanced enough → both off

    analogWrite(PIN_VIBE_LEFT, 0);

    analogWrite(PIN_VIBE_RIGHT, 0);

    return;

  }

  // Map imbalance outside the dead zone → 0..1 intensity

  float intensity = (delta - DZ) / (0.5f - DZ);

  if (intensity < 0) intensity = 0;

  if (intensity > 1) intensity = 1;

  int pwm = int(255.0f * intensity);

  // If leftShare is LOW, left arm is lagging → vibrate LEFT motor.

  // If leftShare is HIGH, right arm is lagging → vibrate RIGHT motor.

  if (leftShare < 0.3f) {

    Serial.println("Swing your left arm more!");

    analogWrite(PIN_VIBE_LEFT,  pwm);

    analogWrite(PIN_VIBE_RIGHT, 0);

  } else if (0.3f < leftShare && leftShare < 0.7f) {

    Serial.println("PERFECT!");

  } else {

    Serial.println("Swing your right arm more!");

    analogWrite(PIN_VIBE_LEFT,  0);

    analogWrite(PIN_VIBE_RIGHT, pwm);

  }

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

  if (!mpuRight.begin(ADDR_RIGHT)) {

    Serial.println(F("Right MPU6050 not found!")); while (1) delay(10);

  }

  if (!mpuLeft.begin(ADDR_LEFT)) {

    Serial.println(F("Left MPU6050 not found!")); while (1) delay(10);

  }

  mpuLeft.setGyroRange(MPU6050_RANGE_500_DEG);

  mpuRight.setGyroRange(MPU6050_RANGE_500_DEG);

  mpuLeft.setFilterBandwidth(MPU6050_BAND_21_HZ);

  mpuRight.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateGyro(left);

  calibrateGyro(right);

  lastMicros = micros();

  lastSendTime = millis();

  left.lastSwingTime = 0; left.swingStartMs = 0;

  right.lastSwingTime = 0; right.swingStartMs = 0;

  pinMode(PIN_VIBE_LEFT, OUTPUT);

  pinMode(PIN_VIBE_RIGHT, OUTPUT);

  analogWrite(PIN_VIBE_LEFT, 0);

  analogWrite(PIN_VIBE_RIGHT, 0);

}

void loop() {

  uint64_t now = micros();

  float dt = (now - lastMicros) * 1e-6f;

  lastMicros = now;

  updateArm(left, dt);

  updateArm(right, dt);

  if (millis() - lastSendTime >= 10000) {

    // 10s window: feedback first (uses current window),

    // then send (which resets counters for the next window)

    feedback();

    sendToGoogleSheets();

    lastSendTime = millis();

  }

  delayMicroseconds(1000000 / SAMPLE_RATE_HZ - 200);

}

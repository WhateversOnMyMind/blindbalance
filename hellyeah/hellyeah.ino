#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* ── pins ── */
#define SDA_PIN              21
#define SCL_PIN              22

/* ── addresses ── */
#define ADDR_LEFT            0x68
#define ADDR_RIGHT           0x69

/* ── sampling & calibration ── */
#define SAMPLE_RATE_HZ       250
#define CALIB_TIME_MS        2000

/* ── thresholds ── */
#define GYRO_THRESH_FWD      0.4f
#define GYRO_THRESH_BACK     0.2f
#define MIN_DELTA_THETA      0.12f

#define RADIUS_DEFAULT       0.30f

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
  const char* label;
};

Arm left = { &mpuLeft, 0.0f, RADIUS_DEFAULT, false, 0, 0.0f, 0.0f, 0, "L" };
Arm right = { &mpuRight, 0.0f, RADIUS_DEFAULT, false, 0, 0.0f, 0.0f, 0, "R" };

uint64_t lastMicros = 0;

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
  float dTheta = endAngle - arm.startAngle;
  if (fabsf(dTheta) < MIN_DELTA_THETA) { arm.inSwing = false; return; }

  ++arm.swingCount;
  float arcLen = fabsf(dTheta) * arm.radius;
  Serial.print(F("Swing ")); Serial.print(arm.label);
  Serial.print(F(" #")); Serial.print(arm.swingCount);
  Serial.print(F(" │ Angle: ")); Serial.print(fabsf(dTheta) * 57.29578f, 1);
  Serial.print(F(" ° │ Length: ")); Serial.print(arcLen * 100.0f, 1);
  Serial.println(F(" cm"));

  arm.inSwing = false;
}

void updateArm(Arm &arm, float dt) {
  sensors_event_t a, g, t;
  arm.mpu->getEvent(&a, &g, &t);
  float wy = g.gyro.y - arm.bias;
  arm.angle += wy * dt;

  if (!arm.inSwing) {
    if (wy > GYRO_THRESH_FWD) {
      arm.inSwing = true; arm.dir = +1; arm.startAngle = arm.angle;
    }
    else if (wy < -GYRO_THRESH_BACK) {
      arm.inSwing = true; arm.dir = -1; arm.startAngle = arm.angle;
    }
  }
  else {
    if (arm.dir == +1 && wy < -GYRO_THRESH_BACK)
      printSwing(arm, arm.angle);
    else if (arm.dir == -1 && wy > GYRO_THRESH_FWD)
      printSwing(arm, arm.angle);
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpuLeft.begin(ADDR_LEFT)) {
    Serial.println(F("Left MPU6050 not found!")); while (1) delay(10);
  }
  if (!mpuRight.begin(ADDR_RIGHT)) {
    Serial.println(F("Right MPU6050 not found!")); while (1) delay(10);
  }

  mpuLeft.setGyroRange(MPU6050_RANGE_500_DEG);
  mpuRight.setGyroRange(MPU6050_RANGE_500_DEG);
  mpuLeft.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpuRight.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateGyro(left);
  calibrateGyro(right);

  lastMicros = micros();
}

void loop() {
  uint64_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  updateArm(left, dt);
  updateArm(right, dt);

  delayMicroseconds(1000000 / SAMPLE_RATE_HZ - 200);
}

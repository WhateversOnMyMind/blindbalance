/****************************************************************************************
  Jog Arm-Swing Counter   –  Y-axis version (ESP32 LOLIN-D32  +  MPU6050)
****************************************************************************************/
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* ── pins ── */
#define SDA_PIN              21
#define SCL_PIN              22

/* ── sampling & calibration ── */
#define SAMPLE_RATE_HZ       250        // ≈4 ms loop
#define CALIB_TIME_MS        2000       // 2 s gyro-bias average

/* ── swing thresholds (provided) ── */
#define GYRO_THRESH_FWD      0.4f       // rad/s  forward swing must exceed
#define GYRO_THRESH_BACK     0.2f       // rad/s  backward swing must exceed
#define MIN_DELTA_THETA      0.12f      // rad ≈ 7°  ignore tiny jitters

/* ── geometry ── */
#define RADIUS_DEFAULT       0.30f      // m  elbow→wrist

/* ── globals ── */
Adafruit_MPU6050 mpu;

float gyBias  = 0.0f;                  // Y-axis bias
float radius  = RADIUS_DEFAULT;

bool  inSwing = false;
int   dir     = 0;                     // +1 = forward, –1 = backward
float angle   = 0.0f;                  // integrated θ about Y (rad)
float startAngle = 0.0f;
uint32_t swingCount = 0;

uint64_t lastMicros = 0;

/* ─────────── helpers ─────────── */
void printSwing(float endAngle)
{
  float dTheta = endAngle - startAngle;
  if (fabsf(dTheta) < MIN_DELTA_THETA) { inSwing = false; return; }  // too small

  ++swingCount;
  float arcLen = fabsf(dTheta) * radius;            // m

  Serial.print(F("Swing #"));  Serial.print(swingCount);
  Serial.print(F(" │ Angle: "));  Serial.print(fabsf(dTheta) * 57.29578f, 1);
  Serial.print(F(" ° │ Length: ")); Serial.print(arcLen * 100.0f, 1);
  Serial.println(F(" cm"));
  inSwing = false;
}

void calibrateGyro()
{
  Serial.println(F("\n*** Keep sensor STILL – calibrating (2 s)… ***"));
  delay(400);
  uint32_t t0 = millis();  float sum = 0;  uint32_t n = 0;
  sensors_event_t a, g, t;
  while (millis() - t0 < CALIB_TIME_MS) {
    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.y;  ++n;  delay(4);
  }
  gyBias = sum / n;

  swingCount = 0; angle = 0; inSwing = false; dir = 0;
  Serial.print(F("Gyro-Y bias = ")); Serial.print(gyBias, 6); Serial.println(F(" rad/s"));
}

void serialMenu()
{
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'r') calibrateGyro();
  else if (c == 'd') {
    float r = Serial.parseFloat();
    if (r > 0.05 && r < 1.0) {
      radius = r;
      Serial.print(F("Radius set to ")); Serial.print(radius * 100.0f, 1);
      Serial.println(F(" cm"));
    }
  }
}

/* ─────────── setup ─────────── */
void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!mpu.begin()) { Serial.println(F("MPU6050 not found!")); while (1) delay(10); }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);          // ±500 °/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateGyro();
  lastMicros = micros();

  Serial.println(F("\nCommands:  r = recalibrate  |  d <metres> = set radius\n"));
}

/* ─────────── loop ─────────── */
void loop()
{
  /* Δt */
  uint64_t now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;

  /* IMU */
  sensors_event_t a, g, t;  mpu.getEvent(&a, &g, &t);
  float wy = g.gyro.y - gyBias;          // rad/s   USING Y-AXIS ONLY

  /* integrate θ about Y */
  angle += wy * dt;

  /* FSM */
  if (!inSwing)                              // waiting for stroke start
  {
    if (wy >  GYRO_THRESH_FWD)               // forward stroke begins
      { inSwing = true; dir = +1; startAngle = angle; }
    else if (wy < -GYRO_THRESH_BACK)         // backward stroke begins
      { inSwing = true; dir = -1; startAngle = angle; }
  }
  else                                        // inside a stroke
  {
    if (dir == +1 && wy < -GYRO_THRESH_BACK)      // forward finished
        printSwing(angle);
    else if (dir == -1 && wy >  GYRO_THRESH_FWD)  // backward finished
        printSwing(angle);
  }

  serialMenu();
  delayMicroseconds(1000000 / SAMPLE_RATE_HZ - 200);
}

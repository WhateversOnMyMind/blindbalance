#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <vector>
#include <algorithm>

/* ── WiFi ── */
const char* ssid = "GalaxyS23";
const char* password = "12345678";
const char* scriptURL = "https://script.google.com/macros/s/AKfycbzBvJKEtq9sOr6JkIPRjNUf_4xROrjfYzzAtlElzlUsUg69tv45QoupNosSsXKZQs7l/exec";

/* ── pins ── */
#define SDA_PIN 21
#define SCL_PIN 22
#define PIN_VIBE_LEFT 17
#define PIN_VIBE_RIGHT 16

/* ── addresses ── */
#define ADDR_LEFT 0x69
#define ADDR_RIGHT 0x68

/* ── sampling & calibration ── */
#define SAMPLE_RATE_HZ 250
#define CALIB_TIME_MS 2000

/* ── thresholds (default, updated by priming) ── */
float GYRO_THRESH_FWD = 0.4f;
float GYRO_THRESH_BACK = 0.4f;
#define MIN_DELTA_THETA 0.2f
#define RADIUS_DEFAULT 0.30f
float ACCEL_MAG_THRESH_DELTA = 2.0f;

/* ── swing robustness ── */
#define SWING_MAX_MS 1200
#define WY_IDLE 0.12f
#define END_ZERO_CROSS_MS 180
#define BIAS_ADAPT_ALPHA 0.0008f

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
  uint32_t swingStartMs;
};

Arm left  = { &mpuLeft, 0.0f, RADIUS_DEFAULT, false, 0, 0.0f, 0.0f, 0, 0.0f, "L", 0, 0 };
Arm right = { &mpuRight, 0.0f, RADIUS_DEFAULT, false, 0, 0.0f, 0.0f, 0, 0.0f, "R", 0, 0 };

uint64_t lastMicros = 0;
unsigned long lastSendTime = 0;

/* ── Serial-safe print macros ── */
#define PRINTLN(x) if(Serial) Serial.println(x)
#define PRINT(x) if(Serial) Serial.print(x)

/* ── Calibration ── */
void calibrateGyro(Arm &arm) {
  PRINTLN("Calibrating "+String(arm.label));
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
  PRINT("Bias "+String(arm.label)+" = "); PRINTLN(arm.bias);
}

/* ── Swing print ── */
void printSwing(Arm &arm, float endAngle) {
  uint32_t now = millis();
  if (now - arm.lastSwingTime < 400) return; // debounce

  float dTheta = endAngle - arm.startAngle;
  ++arm.swingCount;
  float arcLen = fabsf(dTheta) * arm.radius;
  arm.swingDistanceSum += arcLen;

  PRINT("Swing "); PRINT(arm.label);
  PRINT(" #"); PRINT(arm.swingCount);
  PRINT(" │ Angle: "); PRINT(fabsf(dTheta)*57.29578f);
  PRINT(" ° │ Length: "); PRINTLN(arcLen*100.0f);

  arm.lastSwingTime = now;
  arm.inSwing = false;
  arm.dir = 0;
}

/* ── Update swing ── */
void updateArm(Arm &arm, float dt) {
  sensors_event_t a, g, t;
  arm.mpu->getEvent(&a, &g, &t);
  float wy_raw = g.gyro.y;
  float wy = wy_raw - arm.bias;
  arm.angle += wy * dt;

  float aMag = sqrtf(a.acceleration.x*a.acceleration.x + a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z);
  bool accelTrigger = fabsf(aMag-9.8f) > ACCEL_MAG_THRESH_DELTA;

  if (!arm.inSwing) {
    if (accelTrigger) {
      if (wy > GYRO_THRESH_FWD) arm.inSwing = true, arm.dir=+1, arm.startAngle=arm.angle, arm.swingStartMs=millis();
      else if (wy < -GYRO_THRESH_BACK) arm.inSwing = true, arm.dir=-1, arm.startAngle=arm.angle, arm.swingStartMs=millis();
    }
  } else {
    bool crossed = (arm.dir==+1 && wy<-GYRO_THRESH_BACK)||(arm.dir==-1 && wy>GYRO_THRESH_FWD);
    uint32_t elapsed = millis() - arm.swingStartMs;
    bool zeroLike = (fabsf(wy)<WY_IDLE)&&(elapsed>END_ZERO_CROSS_MS);
    bool timeout = elapsed>SWING_MAX_MS;
    if(crossed||zeroLike||timeout) printSwing(arm, arm.angle);
  }

  if(fabsf(wy)<WY_IDLE) arm.bias = (1.0f-BIAS_ADAPT_ALPHA)*arm.bias + BIAS_ADAPT_ALPHA*wy_raw;
}

/* ── Send to Google Sheets ── */
void sendToGoogleSheets() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.begin(scriptURL);
  http.addHeader("Content-Type","application/json");
  float left_cm = left.swingDistanceSum*100.0f;
  float right_cm = right.swingDistanceSum*100.0f;
  String json="{";
  json += "\"leftCount\":" + String(left.swingCount) + ",";
  json += "\"leftDistance\":" + String(left_cm,2) + ",";
  json += "\"rightCount\":" + String(right.swingCount) + ",";
  json += "\"rightDistance\":" + String(right_cm,2) + "}";
  PRINTLN("Sending JSON: "+json);
  int code = http.POST(json);
  PRINT("HTTP Response code: "); PRINTLN(code);
  http.end();
  left.swingCount=0; right.swingCount=0; left.swingDistanceSum=0; right.swingDistanceSum=0;
}

/* ── Feedback ── */
void feedback() {
  uint32_t lc=left.swingCount, rc=right.swingCount;
  float ld=left.swingDistanceSum, rd=right.swingDistanceSum;
  if((lc+rc)==0||(ld+rd)==0){ analogWrite(PIN_VIBE_LEFT,0); analogWrite(PIN_VIBE_RIGHT,0); return; }
  float countFracL=float(lc)/float(lc+rc);
  float distFracL=ld/(ld+rd);
  float leftShare=(countFracL+distFracL)/2.0f;
  const float DZ=0.055f;
  float delta=fabs(leftShare-0.5f);
  if(delta<=DZ){ analogWrite(PIN_VIBE_LEFT,0); analogWrite(PIN_VIBE_RIGHT,0); return; }
  float intensity=(delta-DZ)/(0.5f-DZ);
  if(intensity<0) intensity=0; ifanalogWrite(PIN_VIBE_LEFT,0); analogWrite(PIN_VIBE_RIGHT,pwm)(intensity>1) intensity=1;
  int   =int(255.0f*intensity);
  if(leftShare<0.3f){ analogWrite(PIN_VIBE_LEFT,pwm); analogWrite(PIN_VIBE_RIGHT,0); }
  else if(leftShare>0.3f && leftShare<0.7f){}
  else { analogWrite(PIN_VIBE_LEFT,0); analogWrite(PIN_VIBE_RIGHT,pwm); }
}

/* ── Priming phase with 90th percentile ── */
float calculatePercentile(std::vector<float> &data, float percentile){
  if(data.size()==0) return 0;
  std::sort(data.begin(),data.end());
  size_t idx=int(percentile*data.size());
  if(idx>=data.size()) idx=data.size()-1;
  return data[idx];
}

void primingPhase(Arm &arm, uint32_t durationMs){
  std::vector<float> posSamples, negSamples, accelSamples;
  uint32_t start=millis();
  analogWrite(PIN_VIBE_LEFT,255);
  analogWrite(PIN_VIBE_RIGHT,255);
  while(millis()-start<durationMs){
    sensors_event_t a,g,t;
    arm.mpu->getEvent(&a,&g,&t);
    float wy = g.gyro.y - arm.bias;
    float aMag = sqrtf(a.acceleration.x*a.acceleration.x + a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z);
    if(wy>0) posSamples.push_back(wy);
    else if(wy<0) negSamples.push_back(fabs(wy));
    accelSamples.push_back(aMag);
    delay(1000/SAMPLE_RATE_HZ);
  }
  analogWrite(PIN_VIBE_LEFT,0);
  analogWrite(PIN_VIBE_RIGHT,0);
  GYRO_THRESH_FWD = calculatePercentile(posSamples,0.9f);
  GYRO_THRESH_BACK = calculatePercentile(negSamples,0.9f);
  ACCEL_MAG_THRESH_DELTA = fmaxf(2.0f,fabs(calculatePercentile(accelSamples,0.9f)-9.8f));
}

/* ── Motor countdown ── */
void motorCountdown(int times,uint32_t onMs){
  for(int i=0;i<times;i++){
    analogWrite(PIN_VIBE_LEFT,255);
    analogWrite(PIN_VIBE_RIGHT,255);
    delay(onMs);
    analogWrite(PIN_VIBE_LEFT,0);
    analogWrite(PIN_VIBE_RIGHT,0);
    delay(500);
  }
}

/* ── Setup ── */
void setup() {
  if(Serial) Serial.begin(115200);
  Wire.setClock(100000);
  Wire.begin(SDA_PIN,SCL_PIN);

  WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED){ delay(500); }

  if(!mpuRight.begin(ADDR_RIGHT)){ while(1) delay(10); }
  if(!mpuLeft.begin(ADDR_LEFT)){ while(1) delay(10); }

  mpuLeft.setGyroRange(MPU6050_RANGE_500_DEG);
  mpuRight.setGyroRange(MPU6050_RANGE_500_DEG);
  mpuLeft.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpuRight.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateGyro(left);
  calibrateGyro(right);

  pinMode(PIN_VIBE_LEFT,OUTPUT);
  pinMode(PIN_VIBE_RIGHT,OUTPUT);
  analogWrite(PIN_VIBE_LEFT,0);
  analogWrite(PIN_VIBE_RIGHT,0);

  lastMicros=micros();
  lastSendTime=millis();

  primingPhase(left,5000);
  primingPhase(right,5000);
  delay(5000);
  motorCountdown(3,3000);
}

/* ── Loop ── */
void loop() {
  uint64_t now=micros();
  float dt=(now-lastMicros)*1e-6f;
  lastMicros=now;

  updateArm(left,dt);
  updateArm(right,dt);

  if(millis()-lastSendTime>=10000){
    feedback();
    sendToGoogleSheets();
    lastSendTime=millis();
  }

  delayMicroseconds(1000000/SAMPLE_RATE_HZ-200);
}

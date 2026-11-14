/* Standalone motion demo for Composite Board rev C.
 * Open THIS sketch, not the parent Jul27 sketch. See README.md.
 * ESP32-S3-MINI-1 + ICM-42688-P; original pin map from ../Jul27.ino.
 * No external Arduino libraries or filesystem uploader required.
 */
#include <Arduino.h>
#include <SPI.h>
#define DEMO_WIFI 0  // Wired by default. Set to 1 only to enable the optional AP.
#if DEMO_WIFI
#include <WiFi.h>
#include <WebServer.h>
#include "WebAssets.h"
#endif
#include <esp_system.h>

static constexpr int PIN_MISO = 8, PIN_MOSI = 9, PIN_SCLK = 10, PIN_CS = 11;
static constexpr int PIN_VIBE = 14;
static constexpr uint32_t SPI_HZ = 8000000;
static constexpr uint16_t CAPACITY = 128;
static const char* AP_PASSWORD = "motiondemo";
SPIClass imuSPI(FSPI);
#if DEMO_WIFI
WebServer server(80);
#endif

struct ImuSample {
  uint32_t seq, us;
  float ax, ay, az, gx, gy, gz, temp;
};
ImuSample history[CAPACITY], outgoing[CAPACITY];
portMUX_TYPE historyMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t newestSeq = 0, bootId = 0;
uint16_t historyCount = 0;
bool imuReady = false;
const char* imuError = "IMU not initialized";

void writeRegister(uint8_t reg, uint8_t value) {
  imuSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_CS, LOW);
  imuSPI.transfer(reg & 0x7f);
  imuSPI.transfer(value);
  digitalWrite(PIN_CS, HIGH);
  imuSPI.endTransaction();
  delayMicroseconds(200);
}
void readRegisters(uint8_t reg, uint8_t* bytes, size_t count) {
  imuSPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(PIN_CS, LOW);
  imuSPI.transfer(reg | 0x80);
  for (size_t i = 0; i < count; i++) bytes[i] = imuSPI.transfer(0);
  digitalWrite(PIN_CS, HIGH);
  imuSPI.endTransaction();
}
uint8_t readRegister(uint8_t reg) {
  uint8_t value;
  readRegisters(reg, &value, 1);
  return value;
}
bool initializeIMU() {
  writeRegister(0x76, 0x00);  // register bank 0
  writeRegister(0x11, 0x01);  // soft reset
  delay(10);
  uint8_t who = readRegister(0x75);
  Serial.printf("ICM-42688-P WHO_AM_I: 0x%02X (expected 0x47)\n", who);
  if (who != 0x47) { imuError = "IMU not found: check SPI pins 8/9/10/11 and power"; return false; }
  writeRegister(0x4f, 0x06);  // +/-2000 dps, 1 kHz ODR (same as Jul27.ino)
  writeRegister(0x50, 0x06);  // +/-16 g, 1 kHz ODR
  writeRegister(0x4e, 0x0f);  // accelerometer + gyro low-noise mode
  delay(100);
  if (readRegister(0x4f) != 0x06 || readRegister(0x50) != 0x06 || (readRegister(0x4e) & 0x0f) != 0x0f) {
    imuError = "IMU configuration readback failed";
    return false;
  }
  return true;
}
void sampleTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    uint8_t raw[14];
    readRegisters(0x1d, raw, sizeof(raw));
    int16_t v[7];
    bool valid = true;
    for (int i = 0; i < 7; i++) {
      v[i] = static_cast<int16_t>((uint16_t(raw[2*i]) << 8) | raw[2*i+1]);
      if (v[i] == INT16_MIN) valid = false;  // invalid sensor sample sentinel
    }
    if (valid) {
      ImuSample s;
      s.us = micros();
      s.temp = v[0]/132.48f + 25.0f;
      s.ax = v[1]/2048.0f; s.ay = v[2]/2048.0f; s.az = v[3]/2048.0f;
      s.gx = v[4]/16.4f; s.gy = v[5]/16.4f; s.gz = v[6]/16.4f;
      portENTER_CRITICAL(&historyMux);
      s.seq = ++newestSeq;
      history[(s.seq-1) % CAPACITY] = s;
      if (historyCount < CAPACITY) ++historyCount;
      portEXIT_CRITICAL(&historyMux);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(4));  // 250 Hz, independent of HTTP
  }
}
String samplePacket(uint32_t after, uint32_t& latest) {
  if (!imuReady) {
    return String("{\"error\":\"") + imuError + "\"}";
  }
  uint16_t count = 0;
  bool dropped = false;
  portENTER_CRITICAL(&historyMux);
  latest = newestSeq;
  if (historyCount) {
    uint32_t oldest = latest-historyCount+1;
    // A new viewer starts at the newest sample; subsequent calls drain history.
    uint32_t first = (after == 0 || after > latest) ? latest : after+1;
    if (first < oldest) { first = oldest; dropped = true; }
    for (uint32_t seq = first; seq <= latest && count < CAPACITY; ++seq) {
      outgoing[count++] = history[(seq-1) % CAPACITY];
    }
  }
  portEXIT_CRITICAL(&historyMux);
  String json;
  json.reserve(160 + count*110);
  json = "{\"boot\":" + String(bootId) + ",\"latest\":" + String(latest);
  json += ",\"dropped\":" + String(dropped ? "true" : "false") + ",\"samples\":[";
  for (uint16_t i = 0; i < count; i++) {
    const ImuSample& s = outgoing[i];
    char row[180];
    snprintf(row, sizeof(row), "%s[%lu,%lu,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.2f]",
      i ? "," : "", (unsigned long)s.seq, (unsigned long)s.us,
      s.ax, s.ay, s.az, s.gx, s.gy, s.gz, s.temp);
    json += row;
  }
  json += "]}";
  return json;
}
#if DEMO_WIFI
void sendSamples() {
  server.sendHeader("Cache-Control", "no-store");
  uint32_t latest=0;
  String json=samplePacket(strtoul(server.arg("after").c_str(),nullptr,10),latest);
  server.send(imuReady?200:503,"application/json",json);
}
#endif
void setup() {
  // Set motor input low before radio or IMU initialization.
  pinMode(PIN_VIBE, OUTPUT);
  digitalWrite(PIN_VIBE, LOW);
  Serial.begin(460800);
  delay(400);
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  imuSPI.begin(PIN_SCLK, PIN_MISO, PIN_MOSI, -1);
  imuReady = initializeIMU();
  bootId = esp_random();
  if (imuReady && xTaskCreate(sampleTask, "imu-250hz", 4096, nullptr, 2, nullptr) != pdPASS) {
    imuReady = false;
    imuError = "Could not start IMU sampling task";
  }
#if DEMO_WIFI
  char apName[32];
  snprintf(apName, sizeof(apName), "BoardMotion-%04X", (unsigned)(ESP.getEfuseMac() & 0xffff));
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  if (!WiFi.softAP(apName, AP_PASSWORD)) {
    Serial.println("Wi-Fi AP failed. Reset the board to retry.");
    return;
  }
  server.on("/", HTTP_GET, [] { server.send_P(200, "text/html; charset=utf-8", ASSET_INDEX); });
  server.on("/style.css", HTTP_GET, [] { server.send_P(200, "text/css", ASSET_STYLE); });
  server.on("/motion.js", HTTP_GET, [] { server.send_P(200, "text/javascript", ASSET_MOTION); });
  server.on("/renderer.js", HTTP_GET, [] { server.send_P(200, "text/javascript", ASSET_RENDERER); });
  server.on("/app.js", HTTP_GET, [] { server.send_P(200, "text/javascript", ASSET_APP); });
  server.on("/serial.js", HTTP_GET, [] { server.send_P(200, "text/javascript", ASSET_SERIAL); });
  server.on("/samples", HTTP_GET, sendSamples);
  server.onNotFound([] { server.send(404, "text/plain", "Not found"); });
  server.begin();
  Serial.printf("\nJoin Wi-Fi: %s\nPassword: %s\nOpen http://%s/\n", apName, AP_PASSWORD, WiFi.softAPIP().toString().c_str());
#else
  Serial.println("USB motion stream ready at 460800 baud. Wi-Fi disabled.");
#endif
  Serial.println("Keep the board still for 2 seconds after opening the page.");
  if (!imuReady) Serial.println(imuError);
}
void loop() {
#if DEMO_WIFI
  server.handleClient();
#endif
  static uint32_t lastSerialMs=0, serialAfter=0;
  static bool serialNeedsBoundary=false;
  // Never block the sampling task on a browser/USB reader. UART needs 460800
  // baud to carry 250 Hz JSON; native USB CDC uses its own USB transfer speed.
  if (Serial && millis()-lastSerialMs >= (imuReady?20UL:1000UL)) {
    lastSerialMs=millis();
    // Resynchronize JSON lines after a partial write before retrying history.
    if (serialNeedsBoundary) {
      if (Serial.write('\n') != 1) return;
      serialNeedsBoundary=false;
    }
    uint32_t latest=serialAfter;
    String packet=samplePacket(serialAfter,latest);
    if (Serial.println(packet) == packet.length()+2) serialAfter=latest;
    else serialNeedsBoundary=true;
  }
  delay(1);
}

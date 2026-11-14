#include <Wire.h>

#define SDA_PIN     5
#define SCL_PIN     6
#define WHO_AM_I    0x00      // Bank 0 register
#define EXPECTED_ID 0xEA      // ICM-20948 WHO_AM_I value

bool readRegister(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  value = Wire.read();
  return true;
}

void checkDevice(uint8_t addr) {
  // 1. Does anything ACK at this address?
  Wire.beginTransmission(addr);
  if (Wire.endTransmission() != 0) {
    Serial.printf("0x%02X: no ACK (nothing connected)\n", addr);
    return;
  }

  // 2. Read WHO_AM_I
  uint8_t id;
  if (!readRegister(addr, WHO_AM_I, id)) {
    Serial.printf("0x%02X: ACK but register read failed\n", addr);
    return;
  }

  if (id == EXPECTED_ID)
    Serial.printf("0x%02X: ICM-20948 found! (WHO_AM_I = 0xEA) ✔\n", addr);
  else
    Serial.printf("0x%02X: device present, but WHO_AM_I = 0x%02X — not an ICM-20948\n", addr, id);
}

void setup() {
  Serial.begin(115200);
  delay(1000);                 // give the serial monitor time to attach

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);       // 400 kHz fast mode

  Serial.println("\n=== ICM-20948 I2C Checker ===");
  Serial.printf("SDA = GPIO%d, SCL = GPIO%d\n\n", SDA_PIN, SCL_PIN);
}

void loop() {
  checkDevice(0x68);           // AD0 low
  checkDevice(0x69);           // AD0 high
  Serial.println("-----------------------------");
  delay(2000);                 // re-check every 2 seconds
}

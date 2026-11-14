/* Composite Board rev. C / piezo version: standalone hardware check.
 * Board: ESP32S3 Dev Module, USB CDC On Boot: Enabled.
 * Serial Monitor: 115200 baud. No extra libraries are required.
 * Repeats: I2C scan, IMU identity check, 1/2/4 kHz beeps on GPIO14.
 * Expected: I2C 0x30 (MMC5983MA), SPI WHO_AM_I 0x47 (ICM-42688-P).
 * The SPI IMU does not appear in the I2C scan.
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

constexpr uint8_t PIEZO = 14;
constexpr uint8_t SDA_PIN = 4, SCL_PIN = 5;
constexpr uint8_t MISO_PIN = 8, MOSI_PIN = 9, SCK_PIN = 10, CS_PIN = 11;
SPIClass imu(FSPI);
bool i2cReady = false;
bool piezoReady = false;

void scanI2c() {
  Serial.println("\nI2C scan: SDA=4, SCL=5 (100 kHz)");
  if (!i2cReady) {
    Serial.println("FAIL: I2C bus initialization failed.");
    return;
  }
  unsigned found = 0, errors = 0;
  // Exclude I2C's reserved addresses.
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    Wire.beginTransmission(address);
    uint8_t result = Wire.endTransmission();
    if (result == 0) {
      ++found;
      Serial.printf("  Found 0x%02X%s\n", address,
                    address == 0x30 ? " (expected MMC5983MA address)" : "");
    } else if (result != 2) {  // Address NACK just means no device there.
      ++errors;
      Serial.printf("  Bus error %u at 0x%02X\n", result, address);
    }
  }
  Serial.printf("I2C: %u device(s), %u bus error(s).\n", found, errors);
}

void checkImu() {
  imu.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  digitalWrite(CS_PIN, LOW);
  imu.transfer(0x76);  // REG_BANK_SEL: select bank 0.
  imu.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(200);
  digitalWrite(CS_PIN, LOW);
  imu.transfer(0x75 | 0x80);  // Read WHO_AM_I.
  uint8_t identity = imu.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  imu.endTransaction();
  Serial.printf("SPI IMU WHO_AM_I: 0x%02X (expected 0x47) -- %s\n",
                identity, identity == 0x47 ? "PASS" : "FAIL");
}

void beepPitches() {
  if (!piezoReady) {
    Serial.println("FAIL: piezo PWM initialization failed.");
    return;
  }
  const uint16_t pitches[] = {1000, 2000, 4000};
  for (uint16_t hz : pitches) {
    Serial.printf("Piezo GPIO14: %u Hz, 200 ms\n", hz);
    if (ledcWriteTone(PIEZO, hz) == 0) {
      Serial.println("FAIL: unable to start tone.");
    }
    delay(200);
    ledcWriteTone(PIEZO, 0);  // Silence while retaining LEDC pin ownership.
    delay(250);
  }
}

void setup() {
  pinMode(PIEZO, OUTPUT);
  digitalWrite(PIEZO, LOW);
  Serial.begin(115200);
  delay(1500);  // Brief USB connection time; runs even without Serial Monitor.
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  imu.begin(SCK_PIN, MISO_PIN, MOSI_PIN, -1);
  i2cReady = Wire.begin(SDA_PIN, SCL_PIN, 100000);
  Wire.setTimeOut(25);
  piezoReady = ledcAttach(PIEZO, 4000, 8);
  if (piezoReady) ledcWriteTone(PIEZO, 0);
  Serial.println("\nComposite Board piezo: scanner + pitch test");
}

void loop() {
  scanI2c();
  checkImu();
  beepPitches();
  Serial.println("Pausing 4 seconds, then repeating...");
  delay(4000);
}

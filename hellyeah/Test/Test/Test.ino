
#define PIN_VIBE_LEFT 17
#define PIN_VIBE_RIGHT 16

void setup() {
  
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(PIN_VIBE_RIGHT, 255);
  delay(3000);
  analogWrite(PIN_VIBE_RIGHT, 0);
  delay(1000);
  analogWrite(PIN_VIBE_LEFT, 255);
  delay(3000);
  analogWrite(PIN_VIBE_LEFT, 0);
  delay(1000);
  
}

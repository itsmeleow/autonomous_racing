#include <Arduino.h>
#include <ESP32Servo.h>

Servo throttle;
Servo steering;

constexpr int NEUTRAL_US = 1500;
constexpr int TEST_US = 1565;  // test value

void setup() {
  Serial.begin(115200);

  throttle.setPeriodHertz(50);
  throttle.attach(D6, 1000, 2000);
  throttle.writeMicroseconds(NEUTRAL_US);

  steering.setPeriodHertz(50);
  steering.attach(D5, 1000, 2000);
  steering.writeMicroseconds(1500);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'r') {
      throttle.writeMicroseconds(TEST_US);
      Serial.println("Running at 1565 us. Send s to stop.");
    } else if (command == 's') {
      throttle.writeMicroseconds(NEUTRAL_US);
      Serial.println("Neutral: 1500 us.");
    }
  }

  delay(1);
}


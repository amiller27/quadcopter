#include <Servo.h>

Servo esc;
int val = 2000;

void setup() {
  Serial.begin(115200);
  esc.attach(3);
}

void loop() {
  if (Serial.available()) {
    val = Serial.parseInt();
    Serial.println(val);
  }
  esc.writeMicroseconds(val);
  delay(10);
}

#include "imu.h"

Imu* imu;
Orientation o;

void setup() {
  imu = new Imu();
  Serial.begin(9600);
}

void loop() {
  imu->GetOrientation(o);
  Serial.print("Heading: ");
  Serial.print(o.heading);
  Serial.print("\tPitch: ");
  Serial.print(o.attitude);
  Serial.print("\tRoll: ");
  Serial.println(o.bank);
  delay(50);
}

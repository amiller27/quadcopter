#include "rc_receiver.h"
#include "controller.h"
#include "imu.h"
#include "radio_controller.h"
//#include "gps_controller.h"

RcReceiver* receiver;
OperationMode mode;
RcCommands commands;

Controller* controller;

RadioController* radio_controller;
//GpsController* gps_controller;

Imu* imu;

const int voltagePin = A2;
const int ledPin = 13;

int ledTimeOn;  //in us
int ledTimeOff; //in us

bool successful_setup;

int current_time;
uint16_t i = 0;

void setup() {
  // Serial.begin(9600);
  receiver = RcReceiver::Create();
  imu = new Imu(successful_setup);
  controller = new Controller(imu);
  radio_controller = new RadioController(controller, receiver);
  //gps_controller = GpsController::Create(controller, imu);
  pinMode(13, OUTPUT);
  current_time = millis();
}

void loop() {
  if (i%100 == 0) {
    int voltage = map(analogRead(voltagePin), 0, 1024, 0, 12600); //in mV
    if (voltage < 9000) { // low voltage
      ledTimeOn = 500;
      ledTimeOff = 1000;
    } else if (!successful_setup) { // error 
      ledTimeOn = 100;
      ledTimeOff = 100;
    } else { // normal operation
      ledTimeOn = 1000;
      ledTimeOff = 0;
    }
  }

  if (i%10 == 0) {
    receiver->Update();
    receiver->GetMode(mode);
    radio_controller->Update();
  }

  imu->UpdateOrientation(i%10 == 0);

/*
  Serial.print(mode);
  Serial.print(F("\t"));
  Serial.print(commands.aggressiveness, 10);
  Serial.print(F("\t"));
  Serial.print(commands.throttle, 10);
  Serial.print(F("\t"));
  Serial.print(commands.yaw, 10);
  Serial.print(F("\t"));
  Serial.print(commands.attitude, 10);
  Serial.print(F("\t"));
  Serial.print(commands.bank, 10);
  Serial.print(F("\t"));
*/

  controller->Update();
  //Serial.println();
  //Serial.print("Voltage:  ");
  //Serial.println(voltage);
  //int new_time = millis();
  //int dt = new_time - current_time;
  //current_time = new_time;
  //Serial.print("dt:  ");
  //Serial.println(dt);
  if (i%100 == 0) {
    int new_time = millis();
    int dt = new_time - current_time;
    current_time = new_time;
    Serial.println(dt);
  }
  i++;
}

void updateLed() {
  if (millis() % (ledTimeOn + ledTimeOff) < ledTimeOn) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

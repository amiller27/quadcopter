#include "rc_receiver.h"
#include "controller.h"
#include "imu.h"
#include "radio_controller.h"
#include "gps_controller.h"

RcReceiver* receiver;
OperationMode mode;
RcCommands commands;

Controller* controller;

RadioController* radio_controller;
GpsController* gps_controller;

Imu* imu;

void setup() {
  Serial.begin(9600);
  receiver = RcReceiver::Create();
  imu = new Imu();
  controller = new Controller(imu);
  radio_controller = new RadioController(controller, receiver);
  gps_controller = GpsController::Create(controller, imu);
}

void loop() {
  receiver->Update();
  receiver->GetMode(mode);
  receiver->GetCommands(commands);

  imu->UpdateAll();

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

  radio_controller->Update();
  controller->Update();

  Serial.println();
}
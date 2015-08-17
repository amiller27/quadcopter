#include "rc_receiver.h"
<<<<<<< Updated upstream
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
=======
#include "CustomServo.h"

RcReceiver* recevier;

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

void setup() {
  receiver = RcReceiver::Create();
  esc1.attach(8);
  esc2.attach(9);
  esc3.attach(10);
  esc4.attach(11);
}

RcCommands commands;
void loop() {
  receiver->Update();
  receiver->GetCommands(commands);
  int throttle;
  if (commands.throttle < .5) {
    throttle = 1000;
  }
  else {
    throttle = 2000;
  }
  esc1.writeMicroseconds(throttle);
  esc2.writeMicroseconds(throttle);
  esc3.writeMicroseconds(throttle);
  esc4.writeMicroseconds(throttle);
}
>>>>>>> Stashed changes

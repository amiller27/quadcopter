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

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void* __brkval;
int freeRam () 
{
  int v; 
  return ((uint16_t)&v) - (((uint16_t)__brkval == 0) ? ((uint16_t)&__bss_end) : ((uint16_t)__brkval)); 
}

void setup() {
  Serial.begin(9600);
  // delay(1000);
  Serial.println(freeRam());
  // delay(1000);
  receiver = RcReceiver::Create();
  // Serial.println(freeRam());
  // delay(1000);
  imu = new Imu();
  // Serial.println(freeRam());
  // delay(1000);
  controller = new Controller(imu);
  Serial.println(freeRam());
  // delay(1000);
  Serial.println(23);
  // Serial.println(freeRam());
  // delay(1000);
  radio_controller = new RadioController(controller, receiver);
  // Serial.println(freeRam());
  // delay(1000);
  Serial.println(25);
  // Serial.println(freeRam());
  // delay(1000);
  gps_controller = GpsController::Create(controller, imu);
  // Serial.println(freeRam());
  // delay(1000);
  Serial.println(27);
  Serial.println(freeRam());
  // delay(1000);
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
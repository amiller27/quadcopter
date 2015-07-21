#include "controller.h"
#include "gps_controller.h"
#include "imu.h"
#include "radio_controller.h"
#include "rc_receiver.h"

Controller* controller;
GpsController* gps_controller;
Imu* imu;
RadioController* radio_controller;
RcReceiver* receiver;

OperationMode mode;

void setup() {
  imu = new Imu();
  receiver = RcReceiver::Create();

  controller = new Controller(imu);
  
  gps_controller = new GpsController(controller);
  radio_controller = new RadioController(controller, receiver);
}

void loop() {
  //imu->UpdateOrientation();
  receiver->Update();
  receiver->GetMode(mode);
  
  switch (mode) {
    case rc:
      radio_controller->Update();
      break;
    case gps:
      gps_controller->Update();
      break;
  }
}

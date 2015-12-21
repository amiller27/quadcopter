#ifndef QUADCOPTER_CONTROLLER_H_
#define QUADCOPTER_CONTROLLER_H_

#include "imu.h"
#include "util.h"

#include <Arduino.h>
#include <limits.h>
#include "CustomServo.h"

struct ControllerCommands {
  float heading = 0;   //in % 
  float attitude = 0;  //in degrees
  float bank = 0;      //in degrees
  float throttle = 0;  //in % of full throttle
};

class Controller {
 public:
  Controller(Imu* imu);
  ~Controller();

  void SetCommands(ControllerCommands& commands);
  void Update();
  
 private:
  //PID constants
  const static float kP_heading = 0;
  const static float kI_heading = 0;

  const static float kP_attitude = 0.0012;
  const static float kI_attitude = 0.0007;

  const static float kP_bank = 0.0012;
  const static float kI_bank = 0.0007;

  //kI and kP balanced. Both values may have been too low. Return to tuning by
  //gradually increasing both

  //Throttle scaling. 50% throttle scaling is 0.5
  //WARNING: Setting throttle scaling to or close to 1.0 (100%) might 
  //         inhibit manuverability at high throttle values!!!
  const static float kThrottleScaling = 0.75;

  unsigned long dt;
  unsigned long last_frame;
  unsigned long this_frame;

  float heading_error_sum;
  float attitude_error_sum;
  float bank_error_sum;

  // Commands
  ControllerCommands commands_;

  // Orientation
  Imu* imu_;

  // ESC connection constants
  int escFRPin = 8;
  int escFLPin = 9;
  int escBRPin = 10;
  int escBLPin = 11;
  Servo escFR;
  Servo escFL;
  Servo escBR;
  Servo escBL;
  const int kMinPulseWidth = 1000; //in microseconds
  const int kMaxPulseWidth = 2000; //in microseconds
};

#endif

#ifndef QUADCOPTER_CONTROLLER_H_
#define QUADCOPTER_CONTROLLER_H_

#include "imu.h"
#include "util.h"

#include <Arduino.h>
#include <limits.h>
#include "CustomServo.h"

struct ControllerCommands {
  float yaw = 0;   //in degrees per second
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
  const static float kP_yaw = 0.001;
  const static float kI_yaw = 0;

  const static float kP_attitude = 0.001;
  const static float kI_attitude = 0;
  const static float kD_attitude = 0;

  const static float kP_bank = 0.001;
  const static float kI_bank = 0;
  const static float kD_bank = 0;

  const static float kMaxYawITerm = 360;
  const static float kMaxAttitudeITerm = 180;
  const static float kMaxBankITerm = 180;

  //Throttle scaling. 50% throttle scaling is 0.5
  //WARNING: Setting throttle scaling to or close to 1.0 (100%) might 
  //         inhibit manuverability at high throttle values!!!
  const static float kThrottleScaling = 0.8;

  float last_heading_;

  float yaw_error_sum_ = 0;
  float attitude_error_sum_ = 0;
  float bank_error_sum_ = 0;

  float attitude_error_last_ = 0;
  float bank_error_last_ = 0;
  uint32_t last_time_ = 0;

  // Commands
  ControllerCommands commands_;
  ControllerCommands last_commands_;

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
  const static int kMinPulseWidth = 1000; //in microseconds
  const static int kMaxPulseWidth = 2000; //in microseconds
};

#endif

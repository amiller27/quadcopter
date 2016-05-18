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
  float kp_adj = 0; // in between 0 and 1
  float ki_adj = 0;
  bool hold_altitude = false;
};

class Controller {
 public:
  Controller(Imu* imu);
  ~Controller();

  void SetCommands(ControllerCommands& commands);
  void Update();
  
  const static float kZeroThrottleThreshold = 0.05;

 private:
  //PID constants
  const static float kP_yaw = 0;
  const static float kI_yaw = 0;

  const static float kP_attitude = 0.01;
  const static float kI_attitude = 0.0;
  const static float kD_attitude = 0.004;

  const static float kP_bank = 0.008;
  const static float kI_bank = 0.0;
  const static float kD_bank = 0.004;

  const static float kP_altitude = 0;

  const static float kMaxYawITerm = 360;
  const static float kMaxAttitudeITerm = 180;
  const static float kMaxBankITerm = 180;

  //Throttle scaling. 50% throttle scaling is 0.5
  //WARNING: Setting throttle scaling to or close to 1.0 (100%) might 
  //         inhibit manuverability at high throttle values!!!
  const static float kThrottleScaling = 0.8;

  float last_heading_;
  float last_throttle_ = 0;
  float altitude_setpoint_ = NAN;

  float yaw_error_sum_ = 0;
  float attitude_error_sum_ = 0;
  float bank_error_sum_ = 0;

  float attitude_error_last_ = 0;
  float bank_error_last_ = 0;
  float dt_ = 0;
  uint32_t this_frame_ = 0;
  uint32_t last_frame_ = 0;

  float attitude_error_diff= 0;
  float bank_error_diff = 0;

  const static int hist_amt = 16;
  int history_index = 0;
  float aed_history[hist_amt];
  float bed_history[hist_amt];
  float aed_history_sum = 0;
  float bed_history_sum = 0;


  // Commands
  ControllerCommands commands_;
  ControllerCommands last_commands_;

  // Orientation
  Imu* imu_;

  // ESC connection constants
  int escFRPin = 9;
  int escFLPin = 8;
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

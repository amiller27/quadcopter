#ifndef QUADCOPTER_CONTROLLER_H_
#define QUADCOPTER_CONTROLLER_H_

#include "imu.h"
#include "util.h"

#include <Servo.h>

struct ControllerCommands {
  float heading = 0;   //in degrees
  float attitude = 0;  //in degrees
  float bank = 0;      //in degrees
  float throttle = 0;  //in % of full throttle
};

class Controller {
 public:
  Controller(Imu* imu);
  ~Controller();

  void Update(ControllerCommands& commands);
  
 private:
  //PID constants
  const static float kP_heading = 1;
  const static float kI_heading = 0;

  const static float kP_attitude = 1;
  const static float kI_attitude = 0;

  const static float kP_bank = 1;
  const static float kI_bank = 0;

  //Maximum throttle cap. 50% throttle cap is 0.5
  //WARNING: Setting throttle cap to 1.0 (100%) might inhibit manuverability
  //         at high throttle values!!!
  const static float kThrottleCap = 0.8;

  //I-Term history
  const static int error_hisory = 2000;
  int error_index;

  float heading_error_values[error_hisory];
  float attitude_error_values[error_hisory];
  float bank_error_values[error_hisory];

  float heading_error_sum;
  float attitude_error_sum;
  float bank_error_sum;

  //orienation
  Imu* imu_;
  Orientation current_orientation_;
  Orientation current_error_;

  //ESC connection constants
  int escFRPin = 0;
  int escFLPin = 1;
  int escBRPin = 2;
  int escBLPin = 3;
  Servo escFR;
  Servo escFL;
  Servo escBR;
  Servo escBL;
  const int kMinPulseWidth = 1000; //in microseconds
  const int kMaxPulseWidth = 1000; //in microseconds
};

#endif

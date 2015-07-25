#ifndef QUADCOPTER_CONTROLLER_H_
#define QUADCOPTER_CONTROLLER_H_

#include "imu.h"
#include "util.h"

struct ControllerCommands {
  float heading = 0;
  float attitude = 0;
  float bank = 0;
  float throttle = 0;
};

class Controller {
 public:
  Controller(Imu* imu);
  ~Controller();

  void Update(ControllerCommands& commands);
  
 private:
  Orientation current_orientation_;
  Imu* imu_;
};

#endif

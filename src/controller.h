#ifndef QUADCOPTER_CONTROLLER_H_
#define QUADCOPTER_CONTROLLER_H_

#include "imu.h"
#include "util.h"

struct Commands {
  float heading = 0;
  float elevation = 0;
  float bank = 0;
  float throttle = 0;
};

class Controller {
 public:
  Controller(Imu* imu);
  ~Controller();

  void Update(Commands& commands);
  
 private:
  Orientation current_orientation_;
  Imu* imu_;
};

#endif

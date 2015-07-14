#ifndef QUADCOPTER_CONTROLLER_H_
#define QUADCOPTER_CONTROLLER_H_

#include "imu.h"
#include "util.h"

class Controller {
 public:
  Controller(Imu* imu);
  ~Controller();

  void Update(/* type here TBD */);
  
 private:
  Orientation current_orientation_;
  Imu* imu_;
};

#endif

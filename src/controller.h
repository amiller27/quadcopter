#ifndef QUADCOPTER_CONTROLLER_H_
#define QUADCOPTER_CONTROLLER_H_

#include "imu.h"

class Controller {
 public:
  Controller();
  ~Controller();

  void Update(Orientation& target);
 private:
  Orientation current_orientation_;
};

#endif
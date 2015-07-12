#ifndef QUADCOPTER_CONTROLLER_H_
#define QUADCOPTER_CONTROLLER_H_

#include "imu.h"

class Controller {
 public:
  Controller();
  ~Controller();

  void Update(/* we will need a parameter here, some kind of orientation */);
};

#endif
#ifndef QUADCOPTER_RADIO_CONTROLLER_H_
#define QUADCOPTER_RADIO_CONTROLLER_H_

#include "controller.h"

class RadioController {
 public:
  RadioController();
  ~RadioController();

  void Update();
 private:
  Controller* controller_;
};

#endif

#ifndef QUADCOPTER_RADIO_CONTROLLER_H_
#define QUADCOPTER_RADIO_CONTROLLER_H_

#include "controller.h"
#include "rc_receiver.h"

class RadioController {
 public:
  RadioController(Controller* controller, RcReceiver* receiver);
  ~RadioController();

  void Update();
  
 private:
  Controller* controller_;
  RcReceiver* receiver_;
  RcCommands commands_;
};

#endif

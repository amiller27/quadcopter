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

  ControllerCommands output_commands_;

  float desiredHeading;

  unsigned long last_time_ = 0;

  // Extreme Pitch and Roll angles at 100% aggressiveness
  const int kMaxPitchRollAngle = 45;  //in deg
  const int kMaxHeadingChange = 180;   //in deg/s
};

#endif

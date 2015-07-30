#include "radio_controller.h"

RadioController::RadioController(Controller* controller,
                                 RcReceiver* receiver)
    : controller_(controller),
      receiver_(receiver) {}

void RadioController::Update() {
  receiver.Update();
  receiver.GetCommands(commands_);

  ControllerCommands outputCommand;
  outputCommand.heading += 2*(commands_.heading - 0.5) * commands_.aggressiveness *
                           kMaxHeadingChange;
  outputCommand.heading %= 360;
  outputCommand.attitude = 2*(commands_.attitude - 0.5) * commands_.aggressiveness *
                           kMaxPitchRollAngle;
  outputCommand.bank = 2*(commands_.bank - 0.5) * commands_.aggressiveness * 
                       kMaxPitchRollAngle;
  outputCommand.throttle = commands_.throttle;

  controller_->Update(outputCommand);
}

#include "radio_controller.h"

RadioController::RadioController(Controller* controller,
                                 RcReceiver* receiver)
    : controller_(controller),
      receiver_(receiver) {}

void RadioController::Update() {
  receiver_->Update();
  receiver_->GetCommands(commands_);

  unsigned long current_time = micros();
  float dt = (current_time - last_time_) / 1000000.0; // in s
  last_time_ = current_time;

  output_commands_.heading += 2*(commands_.yaw - 0.5) * dt *
                           commands_.aggressiveness * kMaxHeadingChange;
  output_commands_.heading = fmod(output_commands_.heading + 360, 360);
  output_commands_.attitude = 2*(commands_.attitude - 0.5) * commands_.aggressiveness *
                           kMaxPitchRollAngle;
  output_commands_.bank = 2*(commands_.bank - 0.5) * commands_.aggressiveness * 
                       kMaxPitchRollAngle;
  output_commands_.throttle = commands_.throttle;

  controller_->SetCommands(output_commands_);
}

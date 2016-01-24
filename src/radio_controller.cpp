#include "conf.h"
#include "radio_controller.h"

RadioController::RadioController(Controller* controller,
                                 RcReceiver* receiver)
    : controller_(controller),
      receiver_(receiver) {}

void RadioController::Update() {
  receiver_->GetCommands(commands_);

  unsigned long current_time = micros();
  float dt = (current_time - last_time_) / 1000000.0; // in s
  last_time_ = current_time;

  output_commands_.yaw = 2*(commands_.yaw - 0.5) * dt *
                         commands_.aggressiveness * kMaxHeadingChange;
  output_commands_.attitude = 2*(commands_.attitude - 0.5) * commands_.aggressiveness *
                           kMaxPitchRollAngle;
  output_commands_.bank = 2*(commands_.bank - 0.5) * commands_.aggressiveness * 
                       kMaxPitchRollAngle;
  if (commands_.throttle <= Controller::kZeroThrottleThreshold) {
    throttle_hit_zero_ = true;
  }

  output_commands_.aggressiveness = commands_.aggressiveness;

  if (throttle_hit_zero_) {
    output_commands_.throttle = commands_.throttle;
  }

  controller_->SetCommands(output_commands_);

#ifdef DEBUG_RC_COMMANDS
  Serial.print(mode);
  Serial.print(F("\t"));
  Serial.print(commands.aggressiveness, 10);
  Serial.print(F("\t"));
  Serial.print(commands.throttle, 10);
  Serial.print(F("\t"));
  Serial.print(commands.yaw, 10);
  Serial.print(F("\t"));
  Serial.print(commands.attitude, 10);
  Serial.print(F("\t"));
  Serial.print(commands.bank, 10);
  Serial.print(F("\t"));
#endif
}

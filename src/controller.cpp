#include "controller.h"

Controller::Controller(Imu* imu) : imu_(imu) {
  escFR.attach(escFRPin, kMinPulseWidth, kMaxPulseWidth);
  escFL.attach(escFLPin, kMinPulseWidth, kMaxPulseWidth);
  escBR.attach(escBRPin, kMinPulseWidth, kMaxPulseWidth);
  escBL.attach(escBLPin, kMinPulseWidth, kMaxPulseWidth);

  imu_->GetHeading(last_heading_);
}

void Controller::SetCommands(ControllerCommands& commands) {
  last_commands_ = commands_;
  memcpy(&commands_, &commands, sizeof(ControllerCommands));
}

void Controller::Update() {

  //full throttle cut
  if (commands_.throttle <= 0) {
    //power down all motors
    escFR.writeMicroseconds(kMinPulseWidth);
    escFL.writeMicroseconds(kMinPulseWidth);
    escBR.writeMicroseconds(kMinPulseWidth);
    escBL.writeMicroseconds(kMinPulseWidth);
    return;
  }

  Orientation current_orientation_;
  imu_->GetOrientation(current_orientation_);
  
  uint32_t current_time = micros();
  uint32_t dt = current_time - last_time_;
  last_time_ = current_time;

  // update current error
  Orientation current_error;
  float current_yaw_error = commands_.yaw - (current_orientation_.heading - last_heading_) / dt * 1000000;

  current_error.attitude = commands_.attitude - current_orientation_.attitude;
  current_error.bank = commands_.bank - current_orientation_.bank;

  yaw_error_sum_ += current_yaw_error;
  attitude_error_sum_ += current_error.attitude;
  bank_error_sum_ += current_error.bank;

  yaw_error_sum_ = constrain(yaw_error_sum_, -kMaxYawITerm, kMaxYawITerm);
  attitude_error_sum_ = constrain(attitude_error_sum_, -kMaxAttitudeITerm, kMaxAttitudeITerm);
  bank_error_sum_ = constrain(bank_error_sum_, -kMaxBankITerm, kMaxBankITerm);

  // calculate error derivative
  float attitude_error_diff = ((current_error.attitude - attitude_error_last_)
                               + (commands_.attitude - last_commands_.attitude)) / dt;
  float bank_error_diff = ((current_error.bank - bank_error_last_)
                           + (commands_.bank - last_commands_.bank)) / dt;
  attitude_error_last_ = current_error.attitude;
  bank_error_last_ = current_error.bank;

  //determind quad adjustments (in % throttle)
  float thr = commands_.throttle * kThrottleScaling;
  float y_adj = kP_yaw * current_yaw_error
              + kI_yaw * yaw_error_sum_;
  float a_adj = kP_attitude * current_error.attitude
              + kI_attitude * attitude_error_sum_
              - kD_attitude * attitude_error_diff;
  float b_adj = kP_bank * current_error.bank
              + kI_bank * bank_error_sum_
              - kD_bank * bank_error_diff;

  // Serial.print(thr);
  // Serial.print("\t");
  // Serial.print(h_adj);
  // Serial.print("\t");
  // Serial.print(a_adj);
  // Serial.print("\t");
  // Serial.print(b_adj);
  // Serial.print("\t");

  //determine ESC pulse widths
  float escFRVal = mapf(thr * (1 + y_adj + a_adj + b_adj), 0, 1, kMinPulseWidth, kMaxPulseWidth);
  float escFLVal = mapf(thr * (1 - y_adj + a_adj - b_adj), 0, 1, kMinPulseWidth, kMaxPulseWidth);
  float escBRVal = mapf(thr * (1 - y_adj - a_adj + b_adj), 0, 1, kMinPulseWidth, kMaxPulseWidth);
  float escBLVal = mapf(thr * (1 + y_adj - a_adj - b_adj), 0, 1, kMinPulseWidth, kMaxPulseWidth);

  // Serial.print(F("FR:  "));
  // Serial.print(escFRVal);
  // Serial.print(F("\tFL:  "));
  // Serial.print(escFLVal);
  // Serial.print(F("\tBR:  "));
  // Serial.print(escBLVal);
  // Serial.print(F("\tBL:  "));
  // Serial.print(escBLVal);

  escFR.writeMicroseconds(escFRVal);
  escFL.writeMicroseconds(escFLVal);
  escBR.writeMicroseconds(escBRVal);
  escBL.writeMicroseconds(escBLVal);
}

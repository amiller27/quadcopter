#include "controller.h"

#include <float.h>

Controller::Controller(Imu* imu) : imu_(imu) {
  imu->GetHeading(last_heading_);
  
  escFR.attach(escFRPin, kMinPulseWidth, kMaxPulseWidth);
  escFL.attach(escFLPin, kMinPulseWidth, kMaxPulseWidth);
  escBR.attach(escBRPin, kMinPulseWidth, kMaxPulseWidth);
  escBL.attach(escBLPin, kMinPulseWidth, kMaxPulseWidth);

  this_frame_ = 0;
  last_frame_ = 0;
}

void Controller::SetCommands(ControllerCommands& commands) {
  commands_.yaw = commands.yaw;
  commands_.attitude = commands.attitude;
  commands_.bank = commands.bank;
  commands_.throttle = commands.throttle;
  commands_.aggressiveness = commands.aggressiveness;
}

void Controller::Update() {
  //full throttle cut
  if (commands_.throttle <= kZeroThrottleThreshold || commands_.throttle > 1.25) {
    //power down all motors
    escFR.writeMicroseconds(kMinPulseWidth);
    escFL.writeMicroseconds(kMinPulseWidth);
    escBR.writeMicroseconds(kMinPulseWidth);
    escBL.writeMicroseconds(kMinPulseWidth);
    last_frame_ = micros();
    return;
  }

  this_frame_ = micros();
  dt_ = this_frame_ - last_frame_;
  last_frame_ = this_frame_;
  dt_ /= 1000000;

  Orientation current_orientation_;
  imu_->GetOrientation(current_orientation_);

  // update current error
  Orientation current_error;
  float current_yaw_error = commands_.yaw - (current_orientation_.heading - last_heading_) / dt_ * 1000000;

  current_error.attitude = commands_.attitude + current_orientation_.attitude;
  current_error.bank = commands_.bank - current_orientation_.bank;

  yaw_error_sum_ += current_yaw_error * dt_;
  attitude_error_sum_ += current_error.attitude * dt_;
  bank_error_sum_ += current_error.bank * dt_;

  yaw_error_sum_ = constrain(yaw_error_sum_, -kMaxYawITerm, kMaxYawITerm);
  attitude_error_sum_ = constrain(attitude_error_sum_, -kMaxAttitudeITerm, kMaxAttitudeITerm);
  bank_error_sum_ = constrain(bank_error_sum_, -kMaxBankITerm, kMaxBankITerm);

  // calculate error derivative
  float attitude_error_diff = ((current_error.attitude - attitude_error_last_)
                               + (commands_.attitude - last_commands_.attitude)) / dt_;
  float bank_error_diff = ((current_error.bank - bank_error_last_)
                           + (commands_.bank - last_commands_.bank)) / dt_;
  attitude_error_last_ = current_error.attitude;
  bank_error_last_ = current_error.bank;

  float thr;
  if (commands_.hold_altitude) {
    if (altitude_setpoint_ == NAN) {
      imu_->GetAltitude(altitude_setpoint_);
      thr = last_throttle_;
    } else {
      float curr_altitude;
      imu_->GetAltitude(curr_altitude);
      thr = last_throttle_ + kP_altitude * (altitude_setpoint_ - curr_altitude);
    }
  } else {
    altitude_setpoint_ = NAN;
    thr = commands_.throttle * kThrottleScaling;
  }
  last_throttle_ = thr;

  //determind quad adjustments (in % throttle)
  float y_adj = kP_yaw * current_yaw_error
              + kI_yaw * yaw_error_sum_;
  float a_adj = commands_.aggressiveness * kP_attitude * current_error.attitude
              + kI_attitude * attitude_error_sum_
              - kD_attitude * attitude_error_diff;
  float b_adj = commands_.aggressiveness * kP_bank * current_error.bank
              + kI_bank * bank_error_sum_
              - kD_bank * bank_error_diff;


/*
  Serial.print(F("thr: "));
  Serial.print(thr);
  Serial.print("\t");
  Serial.print(F("y_ad: "));
  Serial.print(y_adj);
  Serial.print("\t");
  Serial.print(F("a_ad: "));
  Serial.print(a_adj);
  Serial.print("\t");
  Serial.print(F("b_ad: "));
  Serial.print(b_adj);
  Serial.println();
*/

  //determine ESC pulse widths
  float escFRVal = mapf(thr - y_adj - a_adj - b_adj, 0, 1, kMinPulseWidth, kMaxPulseWidth);
  float escFLVal = mapf(thr + y_adj - a_adj + b_adj, 0, 1, kMinPulseWidth, kMaxPulseWidth);
  float escBRVal = mapf(thr + y_adj + a_adj - b_adj, 0, 1, kMinPulseWidth, kMaxPulseWidth);
  float escBLVal = mapf(thr - y_adj + a_adj + b_adj, 0, 1, kMinPulseWidth, kMaxPulseWidth);

  /*
  Serial.print(F("FR:  "));
  Serial.print(escFRVal);
  Serial.print(F("\tFL:  "));
  Serial.print(escFLVal);
  Serial.print(F("\tBR:  "));
  Serial.print(escBRVal);
  Serial.print(F("\tBL:  "));
  Serial.print(escBLVal);
  Serial.println();
  */

  escFR.writeMicroseconds(escFRVal);
  escFL.writeMicroseconds(escFLVal);
  escBR.writeMicroseconds(escBRVal);
  escBL.writeMicroseconds(escBLVal);
}

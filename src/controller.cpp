#include "controller.h"

Controller::Controller(Imu* imu) : imu_(imu) {
  heading_error_sum = 0;
  attitude_error_sum = 0;
  bank_error_sum = 0;

  escFR.attach(escFRPin);
  escFL.attach(escFLPin);
  escBR.attach(escBRPin);
  escBL.attach(escBLPin);
  this_frame = 0;
  last_frame = 0;
}

void Controller::SetCommands(ControllerCommands& commands) {
  commands_.heading = commands.heading;
  commands_.attitude = commands.attitude;
  commands_.bank = commands.bank;
  commands_.throttle = commands.throttle;
}

void Controller::Update() {

  //full throttle cut
  if (commands_.throttle <= 0.02 || commands_.throttle > 1.25) {
    //power down all motors
    escFR.write(kMinPulseWidth);
    escFL.write(kMinPulseWidth);
    escBR.write(kMinPulseWidth);
    escBL.write(kMinPulseWidth);
    last_frame = micros();
    return;
  }

  this_frame = micros();
  dt = this_frame - last_frame;
  last_frame = this_frame;
  dt /= 1000000;

  Orientation current_orientation_;
  imu_->GetOrientation(current_orientation_);
  
  // update current error
  Orientation current_error;
  current_error.heading = commands_.heading - current_orientation_.heading;
  if (current_error.heading > 180) {
    current_error.heading -= 360;
  }
  else if (current_error.heading < -180) {
    current_error.heading += 360;
  }

  current_error.attitude = commands_.attitude + current_orientation_.attitude;
  current_error.bank = commands_.bank - current_orientation_.bank;

  heading_error_sum += current_error.heading * dt;
  attitude_error_sum += current_error.attitude * dt;
  bank_error_sum += current_error.bank * dt;

  /*
  attitude_error_sum = constrain(attitude_error_sum, -180, 180);
  bank_error_sum = constrain(bank_error_sum, -180, 180);
  */

  //determind quad adjustments (in % throttle)
  float thr = commands_.throttle * kThrottleScaling;
  float h_adj = kP_heading * current_error.heading +
                             kI_heading * heading_error_sum;
  float a_adj = kP_attitude * current_error.attitude +
                              kI_attitude * attitude_error_sum;
  float b_adj = kP_bank * current_error.bank +
                          kI_bank * bank_error_sum;

  //determine ESC pulse widths
 /*
  Serial.print(thr);
  Serial.print("\t");
  Serial.print(h_adj);
  Serial.print("\t");
  Serial.print(a_adj);
  Serial.print("\t");
  Serial.print(b_adj);
  Serial.print("\t");
  */
  float escFRVal = constrain(mapf(thr * (1 - h_adj - a_adj - b_adj), 0, 1, kMinPulseWidth, kMaxPulseWidth), kMinPulseWidth, kMaxPulseWidth);
  float escFLVal = constrain(mapf(thr * (1 + h_adj - a_adj + b_adj), 0, 1, kMinPulseWidth, kMaxPulseWidth), kMinPulseWidth, kMaxPulseWidth);
  float escBRVal = constrain(mapf(thr * (1 + h_adj + a_adj - b_adj), 0, 1, kMinPulseWidth, kMaxPulseWidth), kMinPulseWidth, kMaxPulseWidth);
  float escBLVal = constrain(mapf(thr * (1 - h_adj + a_adj + b_adj), 0, 1, kMinPulseWidth, kMaxPulseWidth), kMinPulseWidth, kMaxPulseWidth);

/*
  Serial.print(F("FR:  "));
  Serial.print(escFRVal);
  Serial.print(F("\tFL:  "));
  Serial.print(escFLVal);
  Serial.print(F("\tBR:  "));
  Serial.print(escBRVal);
  Serial.print(F("\tBL:  "));
  Serial.print(escBLVal);
*/

  escFR.writeMicroseconds(escFRVal);
  escFL.writeMicroseconds(escFLVal);
  escBR.writeMicroseconds(escBRVal);
  escBL.writeMicroseconds(escBLVal);
}

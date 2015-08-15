#include "controller.h"

Controller::Controller(Imu* imu) : imu_(imu) {
  escFR.attach(escFRPin);
  escFL.attach(escFLPin);
  escBR.attach(escBRPin);
  escBL.attach(escBLPin);
}

void Controller::SetCommands(ControllerCommands& commands) {
  commands_.heading = commands.heading;
  commands_.attitude = commands.attitude;
  commands_.bank = commands.bank;
  commands_.throttle = commands.throttle;
}

void Controller::Update() {

  //full throttle cut
  if (commands_.throttle <= 0) {
    //power down all motors
    escFR.write(kMinPulseWidth);
    escFL.write(kMinPulseWidth);
    escBR.write(kMinPulseWidth);
    escBL.write(kMinPulseWidth);
    return;
  }

  Orientation current_orientation_;
  imu_->GetOrientation(current_orientation_);
  
  // update current error
  Orientation current_error;
  current_error.heading = commands_.heading - current_orientation_.heading;
  if (current_error.heading > 180) {
    current_error.heading -= 360;
  }
  else if (current_error.heading < -180) {
    current_error.heading += 180;
  }

  current_error.attitude = commands_.attitude - current_orientation_.attitude;
  current_error.bank = commands_.bank - current_orientation_.bank;

  //update error history
  // heading_error_sum -= heading_error_values[error_index];
  // attitude_error_sum -= attitude_error_values[error_index];
  // bank_error_sum -= bank_error_values[error_index];

  heading_error_sum += current_error.heading;
  attitude_error_sum += current_error.attitude;
  bank_error_sum += current_error.bank;

  //update error history sum
  // heading_error_values[error_index] = current_error.heading;
  // attitude_error_values[error_index] = current_error.attitude;
  // bank_error_values[error_index] = current_error.bank;
  
  //increment index and reset to 0 if necessary
  // (++error_index)%=error_hisory;

  /*
  NOTE: orientation ajustments are independent of throttle. In future, maybe adjustments
        should be proportional to throttle
  */

  //determind quad adjustments (in % throttle)
  float thr = commands_.throttle * kThrottleScaling;
  float h_adj = kP_heading * current_error.heading +
                             kI_heading * heading_error_sum/* / error_hisory*/;
  float a_adj = kP_attitude * current_error.attitude +
                              kI_attitude * attitude_error_sum/* / error_hisory*/;
  float b_adj = kP_bank * current_error.bank +
                          kI_bank * bank_error_sum/* / error_hisory*/;

  //determine ESC pulse widths
  Serial.print(thr);
  Serial.print("\t");
  Serial.print(h_adj);
  Serial.print("\t");
  Serial.print(a_adj);
  Serial.print("\t");
  Serial.print(b_adj);
  Serial.print("\t");
  float escFRVal = constrain(mapf(thr + h_adj + a_adj + b_adj, 0, 1, kMinPulseWidth, kMaxPulseWidth), kMinPulseWidth, kMaxPulseWidth);
  float escFLVal = constrain(mapf(thr - h_adj + a_adj - b_adj, 0, 1, kMinPulseWidth, kMaxPulseWidth), kMinPulseWidth, kMaxPulseWidth);
  float escBRVal = constrain(mapf(thr - h_adj - a_adj + b_adj, 0, 1, kMinPulseWidth, kMaxPulseWidth), kMinPulseWidth, kMaxPulseWidth);
  float escBLVal = constrain(mapf(thr + h_adj - a_adj - b_adj, 0, 1, kMinPulseWidth, kMaxPulseWidth), kMinPulseWidth, kMaxPulseWidth);

  Serial.print(F("FR:  "));
  Serial.print(escFRVal);
  Serial.print(F("\tFL:  "));
  Serial.print(escFLVal);
  Serial.print(F("\tBR:  "));
  Serial.print(escBLVal);
  Serial.print(F("\tBL:  "));
  Serial.print(escBLVal);

  escFR.write(escFRVal);
  escFL.write(escFLVal);
  escBR.write(escBRVal);
  escBL.write(escBLVal);
}

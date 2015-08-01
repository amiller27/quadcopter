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

  imu_->GetOrientation(current_orientation_);
  
  // update current error
  current_error_.heading = commands_.heading - current_orientation_.heading;
  if (current_error_.heading > 180) {
    current_error_.heading -= 360;
  }
  else if (current_error_.heading < -180) {
    current_error_.heading += 180;
  }

  current_error_.attitude = commands_.attitude - current_orientation_.heading;
  current_error_.bank = commands_.bank - current_orientation_.bank;

  //update error history
  heading_error_sum -= heading_error_values[error_index];
  attitude_error_sum -= attitude_error_values[error_index];
  bank_error_sum -= bank_error_values[error_index];

  heading_error_sum += current_error_.heading;
  attitude_error_sum += current_error_.attitude;
  bank_error_sum += current_error_.bank;

  //update error history sum
  heading_error_values[error_index] = current_error_.heading;
  attitude_error_values[error_index] = current_error_.attitude;
  bank_error_values[error_index] = current_error_.bank;
  
  //increment index and reset to 0 if necessary
  (++error_index)%=error_hisory;

  /*
  NOTE: orientation ajustments are independent of throttle. In future, maybe adjustments
        should be proportional to throttle
  */

  //determind quad adjustments (in % throttle)
  float thr = commands_.throttle * kThrottleScaling;
  float h_adj = kP_heading * current_error_.heading +
                             kI_heading * heading_error_sum / error_hisory;
  float a_adj = kP_attitude * current_error_.attitude +
                              kI_attitude * attitude_error_sum / error_hisory;
  float b_adj = kP_bank * current_error_.bank +
                          kI_bank * bank_error_sum / error_hisory;

  //determine ESC pulse widths 
  float escFRVal = map(thr + h_adj + a_adj + b_adj, 0, 100, kMinPulseWidth, kMaxPulseWidth);
  float escFLVal = map(thr - h_adj + a_adj - b_adj, 0, 100, kMinPulseWidth, kMaxPulseWidth);
  float escBRVal = map(thr - h_adj - a_adj + b_adj, 0, 100, kMinPulseWidth, kMaxPulseWidth);
  float escBLVal = map(thr + h_adj - a_adj - b_adj, 0, 100, kMinPulseWidth, kMaxPulseWidth);

  Serial.print("FR:  ");
  Serial.print(escFRVal);
  Serial.print("\tFL:  ");
  Serial.print(escFLVal);
  Serial.print("\tBR:  ");
  Serial.print(escBLVal);
  Serial.print("\tBL:  ");
  Serial.println(escBLVal);

/*
  escFR.write(escFRVal);
  escFL.write(escFLVal);
  escBR.write(escBRVal);
  escBL.write(escBLVal);
*/
}

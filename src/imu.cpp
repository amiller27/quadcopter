#include <Arduino.h>
#include "imu.h"

#include <math.h>
#include <float.h>

Imu::Imu(bool &successful) {

  int error = 0;

  // set up accelerometer
  if (!accel_.begin()) {
    error = 1;
  }

  accel_.setRange(ADXL345_RANGE_16_G);

  // set up gyroscope
  Wire.begin();

  if (!gyro_.init()) {
    // problem with gyro connection
    error = 2;
  }

  gyro_.writeReg(CTRL_REG1, 0x8f); // set 400Hz ODR, power on
  gyro_.writeReg(CTRL_REG5, 0x02); // enable second built-in LPF
  gyro_.writeReg(CTRL_REG4, 0x30); // set 2000 deg/s range

  if (!mag_.begin()) {
    // problem with magnetometer connection
    error = 3;
  }

  if (!baro_.begin()) {
    // problem with barometer connection
    error = 4;
  }

  TWBR = 24;

  successful = true;
  if (error) {
    Serial.print("Error in Imu:Imu(). Error Code: ");
    Serial.println(error);
    successful = false;
  }

  last_sensor_time_ = micros();
  all_data_.orientation.bank=0;
  all_data_.orientation.attitude=0;
  all_data_.orientation.heading=0;
  all_data_.pressure = 0;
  CalibrateMagnetometer(5);
}

void Imu::CalibrateMagnetometer(int cycles) {
  float magX = 0;
  float magY = 0;

  for (int i = 0; i<cycles; i++) {
    mag_.getEvent(&sensor_event_);
    magX += sensor_event_.magnetic.x+kMagnetometerXOffset;
    magY += sensor_event_.magnetic.y+kMagnetometerYOffset;
    Serial.print(F("X: "));
    Serial.print(magX);
    Serial.print(F("\tY: "));
    Serial.println(magY);
  }
  magX /= cycles;
  magY /= cycles;

  heading_offset = -atan2(-magX, magY) * RAD_TO_DEG;
  Serial.print(F("Heading Offset: "));
  Serial.println(heading_offset, 3);
}

void Imu::UpdateAll(bool update_heading) {
  UpdateOrientation(update_heading);
  
  if (millis() - last_baro_time_ > 25 || last_baro_time_ == 0) {
    last_baro_time_ = millis();
    float new_pressure;
    baro_.pollPressure(&new_pressure);
    if (abs(all_data_.pressure - new_pressure) >= 20) {
      all_data_.pressure = new_pressure;
    } else {
      all_data_.pressure = 0.95 * all_data_.pressure + 0.05 * new_pressure;
    }
    //Serial.println(all_data_.pressure);
  }
}

void Imu::UpdateOrientation(bool update_heading) {
  ///////////////////////////// ACCELEROMETER ///////////////////////////////
  // has event.acceleration.(x|y|z) in m/s^2
  accel_.getEvent(&sensor_event_);

  float temp = sensor_event_.acceleration.x;
  sensor_event_.acceleration.x = sensor_event_.acceleration.y + kAccelXOffset;
  sensor_event_.acceleration.y = -1 * temp + kAccelYOffset;
  sensor_event_.acceleration.z += kAccelZOffset;
  // Serial.print(sensor_event_.acceleration.x);
  // Serial.print("\t");
  // Serial.print(sensor_event_.acceleration.y);
  // Serial.print("\t");
  // Serial.print(sensor_event_.acceleration.z);
  // Serial.println();

  float g = sqrt(sq(sensor_event_.acceleration.x) +
                 sq(sensor_event_.acceleration.y) +
                 sq(sensor_event_.acceleration.z));

  float accel_bank = fast_acos(hypot(sensor_event_.acceleration.x,
                                     sensor_event_.acceleration.z) / g) *
                     RAD_TO_DEG;
  if (sensor_event_.acceleration.y > 0) {accel_bank *= -1;}
  //Serial.print("ab: ");
  //Serial.print(accel_bank);

  float accel_attitude = fast_acos(hypot(sensor_event_.acceleration.y,
                                         sensor_event_.acceleration.z) / g) *
                              RAD_TO_DEG;
  if (sensor_event_.acceleration.x < 0) {accel_attitude *= -1;}
  //Serial.print("\taa: ");
  //Serial.print(accel_attitude);



  //////////////////////////////// GYROSCOPE ////////////////////////////////
  // has gyro_.g.(x|y|z), which all need to be converted
  gyro_.read();
  temp = gyro_.g.x;
  gyro_.g.x = gyro_.g.y + kGyroscopeXOffset;
  gyro_.g.y = -1 * temp + kGyroscopeYOffset;
  gyro_.g.z = gyro_.g.z + kGyroscopeZOffset;
  gyro_.g.x = kGyroscopeFilterConst * gyro_.g.x + (1 - kGyroscopeFilterConst) * last_gyro_.x;
  gyro_.g.y = kGyroscopeFilterConst * gyro_.g.y + (1 - kGyroscopeFilterConst) * last_gyro_.y;
  gyro_.g.z = kGyroscopeFilterConst * gyro_.g.z + (1 - kGyroscopeFilterConst) * last_gyro_.z;
  last_gyro_.x = gyro_.g.x;
  last_gyro_.y = gyro_.g.y;
  last_gyro_.z = gyro_.g.z;
  //Serial.print(gyro_.g.x);
  //Serial.print("\t");
  //Serial.print(gyro_.g.y);
  //Serial.print("\t");
  //Serial.print(gyro_.g.z);
  //Serial.println();
  ///////////////////////////////////////////////////////////////////////////  

  Orientation p = all_data_.orientation;

  unsigned long current = micros();
  float dt = current - last_sensor_time_;
  last_sensor_time_ = current;

  dt /= 1000000; //convert dt to seconds for sensor compatibility

  /////////////////////////////// BANK ///////////////////////////////////////
  all_data_.orientation.bank = (1 - kAccelerometerWeight) * (p.bank + kGyroscopeConversionFactor *
                      gyro_.g.x * dt) + kAccelerometerWeight * accel_bank;
  ////////////////////////////////////////////////////////////////////////////
  
  float sBank = fast_sin(p.bank / RAD_TO_DEG);
  float cBank = fast_cos(p.bank / RAD_TO_DEG);

  
  /////////////////////////////// ATTITUDE ///////////////////////////////////
  all_data_.orientation.attitude = (1-kAccelerometerWeight) *
  (p.attitude + kGyroscopeConversionFactor * dt * 
  (gyro_.g.y * cBank + gyro_.g.z * sBank)) + kAccelerometerWeight * accel_attitude;
  ////////////////////////////////////////////////////////////////////////////
 
  float sAttitude = fast_sin(p.attitude / RAD_TO_DEG);
  float cAttitude = fast_cos(p.attitude / RAD_TO_DEG);

  /////////////////////////////// HEADING ////////////////////////////////////
  if (update_heading) {
    // has event.magnetic.(x|y|z) in uT
    mag_.getEvent(&sensor_event_);
    sensor_event_.magnetic.x += kMagnetometerXOffset;
    sensor_event_.magnetic.y += kMagnetometerYOffset;
    sensor_event_.magnetic.z += kMagnetometerZOffset;
    temp = sensor_event_.magnetic.x;
    sensor_event_.magnetic.x = sensor_event_.magnetic.y;
    sensor_event_.magnetic.y = -1 * temp;

    sensor_event_.magnetic.x = kMagFilterConst * sensor_event_.magnetic.x + (1 - kMagFilterConst) * last_mag_.x;
    sensor_event_.magnetic.y = kMagFilterConst * sensor_event_.magnetic.y + (1 - kMagFilterConst) * last_mag_.y;
    sensor_event_.magnetic.z = kMagFilterConst * sensor_event_.magnetic.z + (1 - kMagFilterConst) * last_mag_.z;
    last_mag_.x = sensor_event_.magnetic.x;
    last_mag_.y = sensor_event_.magnetic.y;
    last_mag_.z = sensor_event_.magnetic.z;

    all_data_.orientation.heading = atan2(cBank*sensor_event_.magnetic.y - 
                                          sBank*sensor_event_.magnetic.z,
                                          cAttitude*sensor_event_.magnetic.x +
                                          sBank*sAttitude*sensor_event_.magnetic.y +
                                          cBank*sAttitude*sensor_event_.magnetic.z) * RAD_TO_DEG +
                                          heading_offset;
    if (all_data_.orientation.heading > 180) {
      all_data_.orientation.heading -= 360;
    } else if (all_data_.orientation.heading < -180) {
      all_data_.orientation.heading += 360;
    }
  }
  ////////////////////////////////////////////////////////////////////////////
}

void Imu::GetAllData(ImuData& out) {
  Imu::GetOrientation(out.orientation);
  out.pressure = all_data_.pressure;
}

void Imu::GetAltitude(float& out) {
  out = all_data_.altitude;
}

void Imu::GetHeading(float& out) {
  out = all_data_.orientation.heading;
}

void Imu::GetOrientation(Orientation& out) {
  out.bank = all_data_.orientation.bank + kAccelBankOffset;
  out.attitude = all_data_.orientation.attitude;
  out.heading = all_data_.orientation.heading;
  //Serial.print("\tH:  ");
  //Serial.print(out.heading);
  //Serial.print("\tA:  ");
  //Serial.print(out.attitude);
  //Serial.print("\tB:  ");
  //Serial.print(out.bank);
  //Serial.println();
}


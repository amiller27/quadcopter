#include <Arduino.h>

#include "conf.h"
#include "imu.h"

#include <math.h>
#include <float.h>

Imu::Imu(bool &successful) {
  memset(old_gyro_data_, 0, kGyroFilterSize * sizeof(L3G::vector<int16_t>));
  memset(&last_gyro_average_, 0, sizeof(L3G::vector<int16_t>));

  int error = 0;

  Wire.begin();
  TWBR = 24;

  // set up accelerometer
  if (!accel_.begin()) {
    error = 1;
  }

  accel_.setRange(ADXL345_RANGE_16_G);

  // set up gyroscope
  if (!gyro_.init()) {
    // problem with gyro connection
    error = 2;
  }

  gyro_.writeReg(L3G::CTRL_REG1, 0x8f); // set 400Hz ODR, power on
  gyro_.writeReg(L3G::CTRL_REG5, 0x02); // enable second built-in LPF
  gyro_.writeReg(L3G::CTRL_REG4, 0x30); // set 2000 deg/s range

  if (!mag_.begin()) {
    // problem with magnetometer connection
    error = 3;
  }

  if (!baro_.begin()) {
    // problem with barometer connection
    error = 4;
  }


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
  CalibrateGyro(100);
}

void Imu::CalibrateMagnetometer(int cycles) {
  float magX = 0;
  float magY = 0;

  for (int i = 0; i<cycles; i++) {
    mag_.getEvent(&sensor_event_);
    magX += sensor_event_.magnetic.x+kMagXOffset;
    magY += sensor_event_.magnetic.y+kMagYOffset;
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

void Imu::CalibrateGyro(int cycles) {
  int32_t new_gyroXOffset = 0;
  int32_t new_gyroYOffset = 0;
  int32_t new_gyroZOffset = 0;
  
  Serial.println(F("Gyro calibration:"));
  for (int i = 0; i < cycles; i++) {
    gyro_.read();
    Serial.print(F("x: "));
    Serial.print(gyro_.g.x);
    Serial.print(F("\ty: "));
    Serial.print(gyro_.g.y);
    Serial.print(F("\tz: "));
    Serial.println(gyro_.g.z);
    new_gyroXOffset -= gyro_.g.x;
    new_gyroYOffset -= gyro_.g.y;
    new_gyroZOffset -= gyro_.g.z;
    delay(10);
  }
  new_gyroXOffset /= cycles;
  new_gyroYOffset /= cycles;
  new_gyroZOffset /= cycles;

  Serial.println(F("Gyro offsets:"));
  Serial.print(F("x: "));
  Serial.print(new_gyroXOffset);
  Serial.print(F("\ty: "));
  Serial.print(new_gyroYOffset);
  Serial.print(F("\tz: "));
  Serial.println(new_gyroZOffset);

  gyroXOffset_ = new_gyroXOffset;
  gyroYOffset_ = new_gyroYOffset;
  gyroZOffset_ = new_gyroZOffset;
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

#ifdef DEBUG_ACCEL_RAW
  {
    static int i = 0;
    i++;
    if (i == 50) {
      Serial.print(sensor_event_.acceleration.x);
      Serial.print("\t");
      Serial.print(sensor_event_.acceleration.y);
      Serial.print("\t");
      Serial.print(sensor_event_.acceleration.z);
      Serial.print("\t");
      i = 0;
    }
  }
#endif

  float g = sqrt(sq(sensor_event_.acceleration.x) +
                 sq(sensor_event_.acceleration.y) +
                 sq(sensor_event_.acceleration.z));

  float accel_bank = fast_acos(hypot(sensor_event_.acceleration.x,
                                     sensor_event_.acceleration.z) / g) *
                     RAD_TO_DEG;
  if (sensor_event_.acceleration.y > 0) {accel_bank *= -1;}

  float accel_attitude = fast_acos(hypot(sensor_event_.acceleration.y,
                                         sensor_event_.acceleration.z) / g) *
                              RAD_TO_DEG;
  if (sensor_event_.acceleration.x < 0) {accel_attitude *= -1;}

#ifdef DEBUG_ACCEL_ANGLES
  {
    static int i = 0;
    i++;
    if (i == 50) {
      Serial.print("ab: ");
      Serial.print(accel_bank);
      Serial.print("\taa: ");
      Serial.print(accel_attitude);
      Serial.print("\t");
      i = 0;
    }
  }
#endif

  //////////////////////////////// GYROSCOPE ////////////////////////////////
  // has gyro_.g.(x|y|z), which all need to be converted
  gyro_.read();
  int16_t gyro_temp = gyro_.g.x;
  gyro_.g.x = gyro_.g.y + gyroYOffset_;
  gyro_.g.y = -1 * gyro_temp - gyroXOffset_;
  gyro_.g.z = gyro_.g.z + gyroZOffset_;
  last_gyro_average_.x -= old_gyro_data_[last_data_index_].x;
  last_gyro_average_.y -= old_gyro_data_[last_data_index_].y;
  last_gyro_average_.z -= old_gyro_data_[last_data_index_].z;
  memcpy(&old_gyro_data_[last_data_index_], &gyro_.g, sizeof(L3G::vector<int16_t>));
  last_gyro_average_.x += old_gyro_data_[last_data_index_].x;
  last_gyro_average_.y += old_gyro_data_[last_data_index_].y;
  last_gyro_average_.z += old_gyro_data_[last_data_index_].z;
  last_data_index_ = (last_data_index_ + 1) % kGyroFilterSize;
  memcpy(&gyro_.g, &last_gyro_average_, sizeof(last_gyro_average_));
  ///////////////////////////////////////////////////////////////////////////  

  Orientation p = all_data_.orientation;

  unsigned long current = micros();
  float dt = current - last_sensor_time_;
  last_sensor_time_ = current;

  dt /= 1000000; //convert dt to seconds for sensor compatibility

  /////////////////////////////// BANK ///////////////////////////////////////
  all_data_.orientation.bank = (1 - kAccelerometerWeight) * (p.bank + kGyroConversionFactor *
                      gyro_.g.x * dt) + kAccelerometerWeight * accel_bank;
  ////////////////////////////////////////////////////////////////////////////
  
  float sBank = fast_sin(p.bank / RAD_TO_DEG);
  float cBank = fast_cos(p.bank / RAD_TO_DEG);

  
  /////////////////////////////// ATTITUDE ///////////////////////////////////
  all_data_.orientation.attitude = (1-kAccelerometerWeight) *
  (p.attitude + kGyroConversionFactor * dt *
  (gyro_.g.y * cBank + gyro_.g.z * sBank)) + kAccelerometerWeight * accel_attitude;
  ////////////////////////////////////////////////////////////////////////////
 
  float sAttitude = fast_sin(p.attitude / RAD_TO_DEG);
  float cAttitude = fast_cos(p.attitude / RAD_TO_DEG);

  /////////////////////////////// HEADING ////////////////////////////////////
  if (update_heading) {
    // has event.magnetic.(x|y|z) in uT
    mag_.getEvent(&sensor_event_);
    sensor_event_.magnetic.x += kMagXOffset;
    sensor_event_.magnetic.y += kMagYOffset;
    sensor_event_.magnetic.z += kMagZOffset;
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

#ifdef DEBUG_IMU
  {
    static int i = 0;
    i++;
    if (i == 50) {
      printed = true;
      Serial.print(F("att: "));
      Serial.print(all_data_.orientation.attitude + kAccelBankOffset);
      Serial.print(F("\tbnk: "));
      Serial.print(all_data_.orientation.bank + kAccelAttitudeOffset);
      Serial.print(F("\thdg: "));
      Serial.print(all_data_.orientation.heading);
      Serial.print(F("\t"));
      i = 0;
    }
  }
#endif
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
  out.attitude = all_data_.orientation.attitude + kAccelAttitudeOffset;
  out.heading = all_data_.orientation.heading;
}

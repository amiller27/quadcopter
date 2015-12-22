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

  gyro_.enableDefault();

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

  last_sensor_time = micros();
  all_data_.orientation.bank=0;
  all_data_.orientation.attitude=0;
  all_data_.orientation.heading=0;
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

void Imu::UpdateOrientation(bool update_heading) {
  ///////////////////////////// ACCELEROMETER ///////////////////////////////
  // has event.acceleration.(x|y|z) in m/s^2
  accel_.getEvent(&sensor_event_);

  float temp = sensor_event_.acceleration.x;
  sensor_event_.acceleration.x = sensor_event_.acceleration.y;
  sensor_event_.acceleration.y = -1 * temp;

  float g = sqrt(sq(sensor_event_.acceleration.x) +
                 sq(sensor_event_.acceleration.y) +
                 sq(sensor_event_.acceleration.z));
  if (g==0) {
    g = FLT_MIN;
  }
  
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
  gyro_.g.x = gyro_.g.y;
  gyro_.g.y = -1 * temp;
  ///////////////////////////////////////////////////////////////////////////  

  Orientation p = all_data_.orientation;

  unsigned long current = micros();
  float dt = current - last_sensor_time;
  last_sensor_time = current;

  dt /= 1000000; //convert dt to seconds for sensor compatibility

  if (dt == 0) {
    dt = 0.005;
  }
  //Serial.print(F("dt: "));
  //Serial.println(dt, 5);


  /////////////////////////////// BANK ///////////////////////////////////////
  all_data_.orientation.bank = (1 - accelerometerWeight) * (p.bank + kGyroscopeConversionFactor *
                      gyro_.g.x * dt) + accelerometerWeight * accel_bank;
  ////////////////////////////////////////////////////////////////////////////
  
  float sBank = fast_sin(p.bank / RAD_TO_DEG);
  float cBank = fast_cos(p.bank / RAD_TO_DEG);

  
  /////////////////////////////// ATTITUDE ///////////////////////////////////
  all_data_.orientation.attitude = (1-accelerometerWeight) *
  (p.attitude + kGyroscopeConversionFactor * dt * 
  (gyro_.g.y * cBank + gyro_.g.z * sBank)) + accelerometerWeight * accel_attitude;
  ////////////////////////////////////////////////////////////////////////////
 
  float sAttitude = fast_sin(p.attitude / RAD_TO_DEG);
  float cAttitude = fast_cos(p.attitude / RAD_TO_DEG);

  /////////////////////////////// HEADING ////////////////////////////////////
  if (update_heading) {
    // has event.magnetic.(x|y|z) in uT
    mag_.getEvent(&sensor_event_);
    sensor_event_.magnetic.x+=kMagnetometerXOffset;
    sensor_event_.magnetic.y+=kMagnetometerYOffset;
    sensor_event_.magnetic.z+=kMagnetometerZOffset;
    temp = sensor_event_.magnetic.x;
    sensor_event_.magnetic.x = sensor_event_.magnetic.y;
    sensor_event_.magnetic.y = -1 * temp;

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

void Imu::GetHeading(float& out) {
  out = all_data_.orientation.heading;
}

void Imu::GetOrientation(Orientation& out) {
  out.bank = all_data_.orientation.bank + accel_bank_offset_;
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

void Imu::GetAltitude(float& out) {
  out = all_data_.altitude;
}

void Imu::UpdateAll() {
  UpdateOrientation(true);

  baro_.getEvent(&sensor_event_);
  if (sensor_event_.pressure) {
    all_data_.pressure = sensor_event_.pressure;
    baro_.getTemperature(&(all_data_.temperature));

    all_data_.altitude = baro_.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,
                                            sensor_event_.pressure);
  } else {
    // barometer error
    Serial.println(F("barometer error"));
  }
}

void Imu::GetAllData(ImuData& out) {
  Imu::GetOrientation(out.orientation);
  out.pressure = all_data_.pressure;
  out.temperature = all_data_.temperature;
  out.altitude = all_data_.altitude;
}

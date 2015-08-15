#include "imu.h"

#include <math.h>

Imu::Imu() {

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

  while (error) {
    Serial.println(error);
    delay(100);
  }

  last_sensor_time = micros();
  all_data_.orientation.bank=0;
  all_data_.orientation.attitude=0;
  all_data_.orientation.heading=0;
}

void Imu::UpdateOrientation() {
  ///////////////////////////// ACCELEROMETER ///////////////////////////////
  // has event.acceleration.(x|y|z) in m/s^2
  accel_.getEvent(&sensor_event_);

  float g = pow(pow(sensor_event_.acceleration.x, 2) +
                pow(sensor_event_.acceleration.y, 2) +
                pow(sensor_event_.acceleration.z, 2), 0.5);
  
  float accel_bank = 90 - asin(pow(pow(sensor_event_.acceleration.x, 2) +
                 pow(sensor_event_.acceleration.z, 2), 0.5)/g) *
                         RAD_TO_DEG;
  if (sensor_event_.acceleration.y > 0) {accel_bank *= -1;}


  float accel_attitude = 90 - asin(pow(pow(sensor_event_.acceleration.y, 2) +
                 pow(sensor_event_.acceleration.z, 2), 0.5)/g) *
                         RAD_TO_DEG;
  if (sensor_event_.acceleration.x < 0) {accel_attitude *= -1;}

  /*
  float accel_heading = 90 - asin(pow(pow(sensor_event_.acceleration.x, 2) +
                 pow(sensor_event_.acceleration.y, 2), 0.5)/g) *
                         RAD_TO_DEG;
  if (sensor_event_.acceleration.z > 0) {accel_heading *= -1;}
  */
  ///////////////////////////////////////////////////////////////////////////


  //////////////////////////////// GYROSCOPE ////////////////////////////////
  // has gyro_.g.(x|y|z), which all need to be converted
  gyro_.read();
  ///////////////////////////////////////////////////////////////////////////  

  Orientation p = all_data_.orientation;

  unsigned long current = micros();
  float dt = current - last_sensor_time;
  last_sensor_time = current;

  dt /= 1000000; //convert dt to seconds for sensor compatibility
  //Serial.print(F("dt: "));
  //Serial.println(dt, 5);


  /////////////////////////////// BANK ///////////////////////////////////////
  all_data_.orientation.bank = (1-acclelerometerWeight) * (p.bank + kGyroscopeConversionFactor *
                      gyro_.g.x * dt) + acclelerometerWeight * accel_bank;
  ////////////////////////////////////////////////////////////////////////////

  
  /////////////////////////////// ATTITUDE ///////////////////////////////////
  all_data_.orientation.attitude = (1-acclelerometerWeight) * 
  (p.attitude + kGyroscopeConversionFactor * dt * 
  (gyro_.g.y * cos(p.bank / RAD_TO_DEG) + gyro_.g.z * 
  sin(p.bank / RAD_TO_DEG))) + acclelerometerWeight * accel_attitude;
  ////////////////////////////////////////////////////////////////////////////
 

  /////////////////////////////// HEADING ////////////////////////////////////
  // has event.magnetic.(x|y|z) in uT
  mag_.getEvent(&sensor_event_);
  all_data_.orientation.heading = atan2((sensor_event_.magnetic.y+kMagnetometerYOffset) / cos(p.bank / RAD_TO_DEG), 
                               (sensor_event_.magnetic.x+kMagnetometerXOffset) / cos(p.attitude / RAD_TO_DEG)) *
                               RAD_TO_DEG;
  ////////////////////////////////////////////////////////////////////////////
}

void Imu::GetHeading(float& out) {
  out = all_data_.orientation.heading;
}

void Imu::GetOrientation(Orientation& out) {
  out.bank = all_data_.orientation.bank;
  out.attitude = all_data_.orientation.attitude;
  out.heading = all_data_.orientation.heading;
}

void Imu::UpdateAll() {
  UpdateOrientation();

  baro_.getEvent(&sensor_event_);
  if (sensor_event_.pressure) {
    all_data_.pressure = sensor_event_.pressure;
    baro_.getTemperature(&(all_data_.temperature));

    all_data_.altitude = baro_.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,
                                            sensor_event_.pressure);
  } else {
    // barrometer error
    Serial.println(F("barometer error"));
  }
}

void Imu::GetAllData(ImuData& out) {
  Imu::GetOrientation(out.orientation);
  out.pressure = all_data_.pressure;
  out.temperature = all_data_.temperature;
  out.altitude = all_data_.altitude;
}

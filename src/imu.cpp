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
    Serial.println("Sensor setup error in imu.cpp.");
    Serial.print("error code: ");
    Serial.println(error);
    delay(500);
  }

  last_sensor_time = micros();
  orientation_.bank=0;
  orientation_.attitude=0;
  orientation_.heading=0;
}

void Imu::GetOrientation(Orientation& out) {
  ///////////////////////////// ACCELEROMETER ///////////////////////////////
  // has event.acceleration.(x|y|z) in m/s^2
  accel_.getEvent(&accel_event_);

  //process acceleration information
  float g = pow(pow(accel_event_.acceleration.x, 2) +
                pow(accel_event_.acceleration.y, 2) +
                pow(accel_event_.acceleration.z, 2), 0.5);
  
  float accel_bank = 90 - asin(pow(pow(accel_event_.acceleration.x, 2) +
                 pow(accel_event_.acceleration.z, 2), 0.5)/g) *
                         RAD_TO_DEG;
  if (accel_event_.acceleration.y > 0) {accel_bank *= -1;}


  float accel_attitude = 90 - asin(pow(pow(accel_event_.acceleration.y, 2) +
                 pow(accel_event_.acceleration.z, 2), 0.5)/g) *
                         RAD_TO_DEG;
  if (accel_event_.acceleration.x < 0) {accel_attitude *= -1;}
  ///////////////////////////////////////////////////////////////////////////


  //////////////////////////////// GYROSCOPE ////////////////////////////////
  // has gyro_.g.(x|y|z), which all need to be converted
  gyro_.read();
  ///////////////////////////////////////////////////////////////////////////

  
  ////////////////////////////// MAGNOMETER /////////////////////////////////
  // has event.magnetic.(x|y|z) in uT
  mag_.getEvent(&mag_event_);
  ///////////////////////////////////////////////////////////////////////////
  

  Orientation p = orientation_; //previous orientation

  //update sensor time and calculate elapsed time
  unsigned long current = micros();
  float dt = current - last_sensor_time;
  last_sensor_time = current;

  dt /= 1000000; //convert dt to seconds for sensor compatibility


  /////////////////////////////// BANK ///////////////////////////////////////
  orientation_.bank = (1-acclelerometerWeight) * (p.bank + kGyroscopeConversionFactor *
                      gyro_.g.x * dt) + acclelerometerWeight * accel_bank;
  ////////////////////////////////////////////////////////////////////////////

  
  /////////////////////////////// ATTITUDE ///////////////////////////////////
  orientation_.attitude = (1-acclelerometerWeight) * 
  (p.attitude + kGyroscopeConversionFactor * dt * 
  (gyro_.g.y * cos(p.bank / RAD_TO_DEG) + gyro_.g.z * 
  sin(p.bank / RAD_TO_DEG))) + acclelerometerWeight * accel_attitude;
  ////////////////////////////////////////////////////////////////////////////
 

  /////////////////////////////// HEADING ////////////////////////////////////
  orientation_.heading = atan2((mag_event_.magnetic.y+kMagnetometerYOffset) / cos(p.bank / RAD_TO_DEG), 
                               (mag_event_.magnetic.x+kMagnetometerXOffset) / cos(p.attitude / RAD_TO_DEG)) *
                               RAD_TO_DEG;
  ////////////////////////////////////////////////////////////////////////////


  out.bank = orientation_.bank;
  out.attitude = orientation_.attitude;
  out.heading = orientation_.heading;
}

void Imu::GetAllData(ImuData& out) {
  GetOrientation(out.orientation);

  baro_.getEvent(&baro_event_);
  if (baro_event_.pressure) {
    out.pressure = baro_event_.pressure;
    baro_.getTemperature(&(out.temperature));

    out.altitude = baro_.pressureToAltitude(SENSORS_PRESSURE_SEALEVELHPA,
                                            baro_event_.pressure);
  } else {
    // barrometer error
    Serial.println("barrometer error");
  }
}

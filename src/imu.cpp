#include "imu.h"

#include <math.h>

Imu::Imu() {

  // set up accelerometer
  if (!accel_.begin()) {
    // problem with accelerometer connection
  }

  accel_.setRange(ADXL345_RANGE_16_G);

  // set up gyroscope
  Wire.begin();

  if (!gyro_.init()) {
    // problem with gyro connection
  }

  gyro_.enableDefault();

  if (!mag_.begin()) {
    // problem with magnetometer connection
  }

  if (!baro_.begin()) {
    // problem with barometer connection
  }

  last_sensor_time = micros();
}

void Imu::GetOrientation(Orientation& out) {
  
  ///////////////////////////// ACCELEROMETER ///////////////////////////////
  // has event.acceleration.(x|y|z) in m/s^2
  accel_.getEvent(&accel_event_);
  
  // compute acceleration norm
  float g = pow(pow(accel_event_.acceleration.x, 2) +
		        pow(accel_event_.acceleration.y, 2) +
				pow(accel_event_.acceleration.z, 2), 0.5); 

  float accel_bank = asin(pow(accel_event_.acceleration.y, 2)
		                 pow(accel_event_.acceleration.z, 2), 0.5);

  float accel_attitude = asin(pow(accel_event_.acceleration.x, 2)
		                 pow(accel_event_.acceleration.z, 2), 0.5);

  float accel_heading = asin(pow(accel_event_.acceleration.x, 2)
		                 pow(accel_event_.acceleration.y, 2), 0.5);
  ///////////////////////////////////////////////////////////////////////////


  //////////////////////////////// GYROSCOPE ////////////////////////////////
  // has gyro_.g.(x|y|z), which all need to be converted
  gyro_.read();
  ///////////////////////////////////////////////////////////////////////////

  
  ////////////////////////////// MAGNOMETER /////////////////////////////////
  // has event.magnetic.(x|y|z) in uT
  mag_.getEvent(&mag_event_);
  ///////////////////////////////////////////////////////////////////////////
  

  Orientation previous_orientation = orientation;

  float dt = -1 * last_sensor_time;     //// These two lines are designed   ////
  dt += (last_sensor_time = micros());  //// to require only one micro call ////

  dt /= 1000000; //convert dt to seconds for sensor compatibility


  /////////////////////////////// BANK ///////////////////////////////////////
  orientation_.bank = (1-acclelerometerWeight) * (previous_orientation_.bank + 
	kGyroscopeConversionFactor * gyro_.g.x * dt) + acclelerometerWeight * accel_bank;
  ////////////////////////////////////////////////////////////////////////////

  
  /////////////////////////////// ATTITUDE ///////////////////////////////////
  orientation_.attitude = (1-acclelerometerWeight) * 
	  (previous_orientation_.attitude + kGyroscopeConversionFactor * dt * 
	  (gyro_.g.y * cos(previous_orientation.bank) + gyro_.g.z * 
	  sin(previous_orientation.bank)) + acclelerometerWeight * accel_heading;
  ////////////////////////////////////////////////////////////////////////////
 

  /////////////////////////////// HEADING ////////////////////////////////////
  orientation_.heading = atan(//left off here)
  ////////////////////////////////////////////////////////////////////////////
  

  *out = orientation_;
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
    Serial.println("barrometer error")
  }
}

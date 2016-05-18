#ifndef QUADCOPTER_CONF_H_
#define QUADCOPTER_CONF_H_

#define DEBUG
// #define DEBUG_ACCEL_RAW
// #define DEBUG_ACCEL_ANGLES
// #define DEBUG_IMU
// #define DEBUG_RC_PULSES
#define DEBUG_RC_COMMANDS
// #define DEBUG_PID
// #define DEBUG_OUTPUTS
// #define DEBUG_VOLTAGE
// #define DEBUG_DT

#ifdef DEBUG
extern bool printed;
#endif

#endif

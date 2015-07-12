#ifndef QUADCOPTER_GPS_CONTROLLER_H_
#define QUADCOPTER_GPS_CONTROLLER_H_

#include "controller.h"
#include "util.h"

class GpsController {
 public:
  GpsController(Controller* controller);
  ~GpsController();

  void AddWaypoint(GpsLocation waypoint);
  void AddWaypoints(GpsLocation waypoints[]);

  void Update();

  // operators
  GpsController& operator=(const GpsController& rhs);
 private:
  Controller* controller_;
  GpsLocation waypoints_[];
};

#endif

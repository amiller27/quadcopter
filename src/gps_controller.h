#ifndef QUADCOPTER_GPS_CONTROLLER_H_
#define QUADCOPTER_GPS_CONTROLLER_H_

#include "controller.h"

struct GpsLocation {
  float latitude = 0;
  float longitude = 0;
  // do we want to include altitude?
};

class GpsController {
 public:
  GpsController(Controller* controller);
  ~GpsController();

  // MORE GPS FUNCTIONS WILL BE NEEDED HERE //
  void AddWaypoint(GpsLocation waypoint);
  void AddWaypoints(GpsLocation waypoints[]);
  ////////////////////////////////////////////

  void Update();

 private:
  Controller* controller_;
  GpsLocation waypoints_[];
};

#endif

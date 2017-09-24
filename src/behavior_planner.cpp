#include <iostream>
#include "vehicle.h"
#include "behavior_planner.h"

using namespace std;

string get_next_state(int lane) {
  if (lane == 0) return "LCR";
  else return "LCL";
}

bool is_front_vehicle_too_close(double car_s, int lane, int prev_size, vector<Vehicle> vehicles) {
  bool too_close = false;
  for (int i = 0; i < vehicles.size(); ++i) {
    Vehicle vehicle = vehicles[i];
    // Car is in my lane
    if (lane == vehicle.lane) {
      double vx = vehicle.vx;
      double vy = vehicle.vy;
      double check_speed = vehicle.v;
      double check_car_s = vehicle.s;

      // If using previous points can project s value outward in time.
      // .02 is because the car moves every points every 20ms.
      check_car_s += ((double)prev_size*.02*check_speed);
      // Check s values greater than mine and s gap
      if ((check_car_s > car_s) && ((check_car_s-car_s) < 30)) {
        // Do some logic here, lower reference velocity so we don't 
        // crash into the car in front of us, could 
        // also flag to try to change lanes.
        too_close = true;
      }
    }
  }

  return too_close;
}
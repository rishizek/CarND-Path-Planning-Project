#include <iostream>
#include "vehicle.h"
#include "behavior_planner.h"

using namespace std;

string get_next_state(double car_s, double car_d, double car_speed, int lane, int prev_size, 
                      vector<Vehicle> neighbor_vehicles) {
  // only consider states which can be reached from current FSM state.
  vector<string> possible_successor_states = {"LCL", "KL", "LCR"};
  if (lane == 0) {
    possible_successor_states.erase(possible_successor_states.begin());  // remove LCL
  } else if (lane == 2) {
    possible_successor_states.erase(possible_successor_states.begin()+2);  // remove LCR
  }

  // keep track of the total cost of each state.
  vector<double> costs;
  vector<double> weights = {10, 9, 1, 1, 5};
  
  for (string state : possible_successor_states) {
    //cout << state << endl;
    // calculate the "cost" associated with that trajectory.
    double cost_for_state = 0;

    cost_for_state += weights[0] * collision_with_front_vehicle_cost(car_s, lane, state, prev_size,
                                                                     neighbor_vehicles);
    /* TODO
    cost_for_state += weights[1] * collision_with_left_vehicle_cost(prev_size, future_car_sd_for_state, neighbor_vehicles);
    cost_for_state += weights[2] * collision_with_right_vehicle_cost(prev_size, future_car_sd_for_state, neighbor_vehicles);
    cost_for_state += weights[3] * change_lane_cost(prev_size, future_car_sd_for_state, neighbor_vehicles);
    cost_for_state += weights[4] * collision_with_behind_vehicle_cost(prev_size, future_car_sd_for_state, neighbor_vehicles);
    */
    costs.push_back(cost_for_state);
  }
  //cout << endl;
  // Find the mininum cost state
  string best_next_state;
  double min_cost = 9999999;
  for (int i = 0; i < possible_successor_states.size(); ++i) {
    string state = possible_successor_states[i];
    double cost = costs[i];
    if (cost < min_cost) {
      min_cost = cost;
      best_next_state = state;
    }
  }
  return best_next_state;
}

double collision_with_front_vehicle_cost(double car_s, int lane, string state, int prev_size,
                                         vector<Vehicle> neighbor_vehicles) {
  if (state.compare("LCL") == 0) {
    lane = lane - 1;
  } else if (state.compare("LCR") == 0) {
    lane = lane + 1;
  } 

  for (Vehicle vehicle : neighbor_vehicles) {
    if (lane == vehicle.lane) {
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
        return 1.0;
      }
    }
  }
  return 0.0;
}

bool is_front_vehicle_too_close(double car_s, int lane, int prev_size, vector<Vehicle> neighbor_vehicles) {
  bool too_close = false;
  for (int i = 0; i < neighbor_vehicles.size(); ++i) {
    Vehicle vehicle = neighbor_vehicles[i];
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

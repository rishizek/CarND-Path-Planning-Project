#include <iostream>
#include "vehicle.h"
#include "behavior_planner.h"

using namespace std;

vector<double> generate_future_car_sd(string state, double car_s, double car_d, double car_speed, 
                                      int future_pos_size) {
  vector<double> future_car_sd = {0, 0};
  double car_angle = 0.52;  // 30 degree is about 0.52 radian
  if (state.compare("LCL") == 0) {
    future_car_sd[0] = ((double)future_pos_size*.02*car_speed*cos(-car_angle));
    future_car_sd[1] = car_d - 4; // because lane size is 4
  } else if (state.compare("LCR") == 0) {
    future_car_sd[0] = ((double)future_pos_size*.02*car_speed*cos(car_angle));
    future_car_sd[1] = car_d + 4; // because lane size is 4
  } else if (state.compare("KL") == 0) {
    future_car_sd[0] = ((double)future_pos_size*.02*car_speed);
    future_car_sd[1] = car_d;
  }
  return future_car_sd;
}

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
  vector<vector<double>> costs;
  vector<double> weights = {10, 9, 1, 1, 5};

  int future_pos_size = 30;
  for (string state : possible_successor_states) {
    // future s and d of car
    vector<double> future_car_sd_for_state = generate_future_car_sd(state, car_s, car_d, car_speed, future_pos_size);
    //cout << state << endl;
    // calculate the "cost" associated with that trajectory.
    double cost_for_state = 0;
    /* TODO
    cost_for_state += weights[0] * collision_with_front_vehicle_cost(future_car_sd_for_stete, neighbor_vehicles);
    cost_for_state += weights[1] * collision_with_behind_vehicle_cost(future_car_sd_for_stete, neighbor_vehicles);
    cost_for_state += weights[2] * collision_with_left_vehicle_cost(future_car_sd_for_stete, neighbor_vehicles);
    cost_for_state += weights[3] * collision_with_right_vehicle_cost(future_car_sd_for_stete, neighbor_vehicles);
    cost_for_state += weights[4] * change_lane_cost(future_car_sd_for_stete, neighbor_vehicles);
    */
  }
  //cout << endl;
  if (lane == 2) return possible_successor_states[0];
  else return possible_successor_states[2];
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
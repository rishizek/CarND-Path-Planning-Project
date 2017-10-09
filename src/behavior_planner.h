#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
using namespace std;

string get_next_state(double, double, double, int, int, vector<Vehicle>);
double collision_with_front_vehicle_cost(double, int, string, int, vector<Vehicle>);
bool is_front_vehicle_too_close(double, int, int, vector<Vehicle>);

#endif /* BEHAVIOR_PLANNER_H */

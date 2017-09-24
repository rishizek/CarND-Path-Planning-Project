#ifndef BEHAVIOR_PLANNER_H
#define BEHAVIOR_PLANNER_H

#include <vector>
using namespace std;

string get_next_state(double, double, double, int, int, vector<Vehicle>);
bool is_front_vehicle_too_close(double, int, int, vector<Vehicle>);

#endif /* BEHAVIOR_PLANNER_H */
#include <iostream>
#include "behavior_planner.h"

using namespace std;

string get_next_state(int lane) {
  if (lane == 0) return "LCR";
  else return "LCL";
}

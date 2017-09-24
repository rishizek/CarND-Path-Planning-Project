#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <math.h>
using namespace std;

class Vehicle {
public:

  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
  double v;
  int lane;

  /**
  * Constructor
  */
  Vehicle(vector<double> sensor_fusion_i) {
    id = sensor_fusion_i[0];
    x  = sensor_fusion_i[1];
    y  = sensor_fusion_i[2];
    vx = sensor_fusion_i[3];
    vy = sensor_fusion_i[4];
    s  = sensor_fusion_i[5];
    d  = sensor_fusion_i[6];
    v  = sqrt(vx*vx+vy*vy);
    if (d >= 0 && d < 4) {
      lane = 0;
    } else if (d >= 4 && d < 8) {
      lane = 1;
    } else if (d >= 8 && d < 12) {
      lane = 2;
    } else {
      lane = -1;
    }
  };

};

#endif
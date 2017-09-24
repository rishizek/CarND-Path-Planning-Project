#ifndef VEHICLE_H
#define VEHICLE_H

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
  };

};

#endif
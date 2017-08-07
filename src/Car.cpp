//
// Created by Alexey Simonov on 05/08/2017.
//

#include "Car.h"
#include <math.h>
#include <cassert>

using namespace std;


Car::Car(double x, double y, double yaw, double speed, const Trajectory& prev_traj) {
  _x = x;
  _y = y;
  _yaw = yaw;
  _speed = speed;
  assert(speed>=0);
  _prev_traj = Trajectory(prev_traj.getDt());
  // copy all points to previous trajectory from current car position till the end
  vector<double> tr_x = prev_traj.getX();
  vector<double> tr_y = prev_traj.getY();
  int n = tr_x.size();
  bool found = false;
  for (int i=0; i<n; i++) {
    if (!found) {
      if (tr_x[i] == x && tr_y[i] == y)
        found = true;
    }
    if (found)
      _prev_traj.add(tr_x[i],tr_y[i]);
  }
}


// translate x,y in car coordinates into global coordinates (given car position on the map)
std::vector<double> Car::car2global(double x_car, double y_car) const
{
  double x_map = x_car * cos(_yaw) - y_car * sin(_yaw) + _x;
  double y_map = x_car * sin(_yaw) + y_car * cos(_yaw) + _y;
  return {x_map, y_map};
}

// translate x,y in map coordinates into car coordinates (given car position on the map)
std::vector<double> Car::global2car(double x_map, double y_map) const
{
  double x_car = (x_map - _x) * cos(_yaw) + (y_map - _y) * sin(_yaw);
  double y_car = (y_map - _y) * cos(_yaw) - (x_map - _x) * sin(_yaw);
  return {x_car, y_car};
}

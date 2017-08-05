//
// Created by Alexey Simonov on 05/08/2017.
//

#include "Car.h"

Car::Car(double x, double y, double yaw, double speed, const Trajectory& prev_traj) {
  _x = x;
  _y = y;
  _yaw = yaw;
  _speed = speed;
  _prev_traj = prev_traj;
}



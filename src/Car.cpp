//
// Created by Alexey Simonov on 05/08/2017.
//

#include "Car.h"
#include <math.h>
#include <cassert>
#include "coordinate_utils.h"
#include <iostream>

using namespace std;

Car::Car(int id,
         double x, double y, double yaw,
         double s, double d,
         int lane,
         double speed, double acceleration,
         double target_speed
)
{
  _id = id;
  _x = x;
  _y = y;
  _yaw = yaw;
  _s = s;
  _d = d;
  _lane = lane;
  assert(speed>=0);
  _speed = speed;
  _acceleration = acceleration;
  assert(target_speed>=0);
  _target_speed = target_speed;
  _target_lane = lane;
  _state = "KL"; // default state
  _predictions_dt = -1.0;
}

Car Car::advance(double T)
{
  Car advanced(*this); // use copy
  advanced._speed += _acceleration * T;
  advanced._s += _speed * T + 0.5 * _acceleration * T * T;
  return advanced;
}

// generate predictions
void Car::generate_predictions(double T, double dt)
 {
   assert(dt>0);
   _predictions_dt = dt;
   _predictions.clear();

  for (double t=0; t<=T; t+=dt)
    // assume other cars do not change lanes
    _predictions.push_back({_lane, advance(t).getS()});
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


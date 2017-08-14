//
// Created by Alexey Simonov on 14/08/2017.
//

#include "BehaviourPlanner.h"

using namespace std;

BehaviourPlanner::BehaviourPlanner(int num_lanes, double lane_width, Car ego, std::vector<Car> other_cars) : _ego(ego)
{
  _num_lanes = num_lanes;
  _lane_width = lane_width;
  _other_cars = other_cars;
}

Car BehaviourPlanner::plan(double T, double max_speed, double max_acceleration, double max_jerk)
{
  Car egoPlan = _ego;

  return egoPlan;
}

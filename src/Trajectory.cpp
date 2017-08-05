//
// Created by Alexey Simonov on 05/08/2017.
//

#include "Trajectory.h"
#include <cassert>

using namespace std;

Trajectory::Trajectory() {}

Trajectory::Trajectory(std::vector<double> x, std::vector<double> y) {
  assert(x.size() == y.size());
  _x_vals = x;
  _y_vals = y;
}

std::vector<double> Trajectory::getX() {
  return _x_vals;
}

std::vector<double> Trajectory::getY() {
  return _y_vals;
}

void Trajectory::add(double x, double y) {
  _x_vals.push_back(x);
  _y_vals.push_back(y);
  assert(_x_vals.size() == _y_vals.size());
}

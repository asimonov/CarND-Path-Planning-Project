//
// Created by Alexey Simonov on 05/08/2017.
//

#include "Trajectory.h"
#include <cassert>
#include "coordinate_utils.h"

using namespace std;

Trajectory::Trajectory(double dt)
{
  //assert(dt>=0);
  _dt=dt;
}

Trajectory::Trajectory(std::vector<double> x, std::vector<double> y, double dt) {
  assert(x.size() == y.size());
  assert(dt>=0);
  _x_vals = x;
  _y_vals = y;
  _dt = dt;
  int n = _x_vals.size();
  for (int i=1;i<n;i++) {
    _dist.push_back(    euclidian_distance(_x_vals[i - 1], _y_vals[i - 1], _x_vals[i], _y_vals[i]));
    _heading.push_back( angle             (_x_vals[i - 1], _y_vals[i - 1], _x_vals[i], _y_vals[i]));
  }
}

std::vector<double> Trajectory::getX() const {
  return _x_vals;
}

std::vector<double> Trajectory::getY() const {
  return _y_vals;
}

double Trajectory::getTotalT() const
{
  assert(_dt>0);
  return _dt * _dist.size();
}


std::vector<double> Trajectory::getStartXY() const
{
  assert(_x_vals.size()>0);
  return {_x_vals[0], _y_vals[0]};
}

std::vector<double> Trajectory::getFinalXY() const
{
  int n=_x_vals.size();
  assert(n>0);
  return {_x_vals[n-1], _y_vals[n-1]};
}

double Trajectory::getStartYaw() const
{
  assert(_x_vals.size()>1);
  return _heading[0];
}

double Trajectory::getFinalYaw() const
{
  int n = _x_vals.size();
  assert(_x_vals.size()>1);
  return _heading[n-2];
}

void Trajectory::add(double x, double y) {
  _x_vals.push_back(x);
  _y_vals.push_back(y);
  assert(_x_vals.size() == _y_vals.size());
  int n = _x_vals.size();
  if (n>1) {
    _dist.push_back   ( euclidian_distance(_x_vals[n - 2], _y_vals[n - 2], _x_vals[n - 1], _y_vals[n - 1]));
    _heading.push_back( angle             (_x_vals[n - 2], _y_vals[n - 2], _x_vals[n - 1], _y_vals[n - 1]));
  }
  assert(n==_dist.size()+1);
  assert(n==_heading.size()+1);
}

double Trajectory::getStartSpeed() const
{
  assert(_dt>0);
  double res = 0.0;
  int n = _dist.size();
  if (n > 0)
    res = _dist[0] / _dt;
  return res;
}

double Trajectory::getFinalSpeed() const
{
  assert(_dt>0);
  double res = 0.0;
  int n = _dist.size();
  if (n > 0)
    res = _dist[n-1] / _dt;
  return res;
}

double Trajectory::getStartAcceleration() const
{
  assert(_dt>0);
  double res = 0.0;
  int n = _dist.size();
  if (n > 1)
  {
    double v2 = _dist[1] / _dt;
    double v1 = _dist[0] / _dt;
    res = ( v2 - v1 ) / _dt;
  }
  return res;
}

double Trajectory::getFinalAcceleration() const
{
  assert(_dt>0);
  double res = 0.0;
  int n = _dist.size();
  if (n > 1)
  {
    double v2 = _dist[n-1] / _dt;
    double v1 = _dist[n-2] / _dt;
    res = ( v2 - v1 ) / _dt;
  }
  return res;
}



// recalculate (inplace) assuming constant speed of v (units/sec) and discretisation dt
/*
void Trajectory::respace_at_constant_speed(double dt, double v)
{
  if (!_x_vals.size())
    return;

  vector<double> x_vals_new;
  vector<double> y_vals_new;

  double disc_dist = v*dt;
  double prev_x = _x_vals[0];
  double prev_y = _y_vals[0];
  double next_x;
  double next_y;
  double dist;

  for (int i=1; i<_x_vals.size(); i++)
  {
    next_x = _x_vals[i];
    next_y = _y_vals[i];
    dist = euclidian_distance(prev_x, prev_y, next_x, next_y);
    double num = dist/disc_dist;
    if (num<1.0)
      break;
    double dx = (next_x - prev_x) / num;
    double dy = (next_y - prev_y) / num;
    double x,y;
    for (int j=0; j<num; j++) {
      x_vals_new.push_back(prev_x + j * dx);
      y_vals_new.push_back(prev_y + j * dy);
    }
    prev_x = next_x;
    prev_y = next_y;
  }

  _x_vals = x_vals_new;
  _y_vals = y_vals_new;
}
*/

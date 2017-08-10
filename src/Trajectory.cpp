//
// Created by Alexey Simonov on 05/08/2017.
//

#include "Trajectory.h"
#include <cassert>
#include "coordinate_utils.h"
#include <limits>

using namespace std;

Trajectory::Trajectory()
{
  _dt = -1;
  _total_distance = 0.0;
  _min_speed = numeric_limits<double>::max();
  _max_speed = numeric_limits<double>::min();
  _min_acceleration = numeric_limits<double>::max();
  _max_acceleration = numeric_limits<double>::min();
  _min_jerk = numeric_limits<double>::max();
  _max_jerk = numeric_limits<double>::min();
}

Trajectory::Trajectory(double dt)
{
  _dt=dt;
  _total_distance = 0.0;
  _min_speed = numeric_limits<double>::max();
  _max_speed = numeric_limits<double>::min();
  _min_acceleration = numeric_limits<double>::max();
  _max_acceleration = numeric_limits<double>::min();
  _min_jerk = numeric_limits<double>::max();
  _max_jerk = numeric_limits<double>::min();
}

Trajectory::Trajectory(std::vector<double> x, std::vector<double> y, double dt) {
  assert(x.size() == y.size());
  assert(dt>=0);
  _x_vals = x;
  _y_vals = y;
  _dt = dt;
  int n = _x_vals.size();
  _total_distance = 0.0;
  for (int i=1;i<n;i++) {
    double d = euclidian_distance(_x_vals[i - 1], _y_vals[i - 1], _x_vals[i], _y_vals[i]);
    _dist.push_back(d);
    _total_distance += d;
    double s = d / _dt;
    _speed.push_back(s);
    if (s<_min_speed)
      _min_speed = s;
    if (s>_max_speed)
      _max_speed = s;
    _heading.push_back( angle(_x_vals[i - 1], _y_vals[i - 1], _x_vals[i], _y_vals[i]));
  }
  assert(_x_vals.size()==0 || _x_vals.size()-1==_dist.size());
  assert(_dist.size()==_speed.size());
  assert(_dist.size()==_heading.size());
  n = _speed.size();
  for (int i=1;i<n;i++) {
    double a = (_speed[i] - _speed[i-1]) / _dt;
    _acceleration.push_back( a );
    if (a<_min_acceleration)
      _min_acceleration = a;
    if (a>_max_acceleration)
      _max_acceleration = a;
  }
  assert(_speed.size()==0 || _speed.size()-1==_acceleration.size());
  n = _acceleration.size();
  for (int i=1;i<n;i++) {
    double j = (_acceleration[i] - _acceleration[i-1]) / _dt;
    _jerk.push_back( j );
    if (j<_min_jerk)
      _min_jerk = j;
    if (j>_max_jerk)
      _max_jerk = j;
  }
  assert(_acceleration.size()==0 || _acceleration.size()-1==_jerk.size());
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
    double d = euclidian_distance(_x_vals[n - 2], _y_vals[n - 2], _x_vals[n - 1], _y_vals[n - 1]);
    _dist.push_back   ( d );
    _total_distance += d;
    double v = d / _dt;
    _speed.push_back  ( v );
    if (v<_min_speed)
      _min_speed = v;
    if (v>_max_speed)
      _max_speed = v;
    _heading.push_back( angle             (_x_vals[n - 2], _y_vals[n - 2], _x_vals[n - 1], _y_vals[n - 1]));
  }
  assert(n==_dist.size()+1);
  assert(n==_heading.size()+1);
  n = _speed.size();
  if (n>1) {
    double a = (_speed[n-1] - _speed[n-2]) / _dt;
    _acceleration.push_back( a );
    if (a<_min_acceleration)
      _min_acceleration = a;
    if (a>_max_acceleration)
      _max_acceleration = a;
  }
  assert(n==0 || n-1==_acceleration.size());
  n = _acceleration.size();
  if (n>1) {
    double j = (_acceleration[n-1] - _acceleration[n-2]) / _dt;
    _jerk.push_back( j );
    if (j<_min_jerk)
      _min_jerk = j;
    if (j>_max_jerk)
      _max_jerk = j;
  }
  assert(_acceleration.size()==0 || _acceleration.size()-1==_jerk.size());
}

double Trajectory::getStartSpeed() const
{
  assert(_dt>0);
  double res = 0.0;
  int n = _speed.size();
  if (n)
    res = _speed[0];
  return res;
}

double Trajectory::getFinalSpeed() const
{
  assert(_dt>0);
  double res = 0.0;
  int n = _speed.size();
  if (n)
    res = _speed[n-1];
  return res;
}

double Trajectory::getStartAcceleration() const
{
  assert(_dt>0);
  double res = 0.0;
  int n = _acceleration.size();
  if (n)
    res = _acceleration[0];
  return res;
}

double Trajectory::getFinalAcceleration() const
{
  assert(_dt>0);
  double res = 0.0;
  int n = _acceleration.size();
  if (n)
    res = _acceleration[n-1];
  return res;
}

double Trajectory::getStartJerk() const
{
  assert(_dt>0);
  double res = 0.0;
  int n = _jerk.size();
  if (n)
    res = _jerk[0];
  return res;
}

double Trajectory::getFinalJerk() const
{
  assert(_dt>0);
  double res = 0.0;
  int n = _jerk.size();
  if (n)
    res = _jerk[n-1];
  return res;
}

double Trajectory::getMinSpeed() const
{
  assert(_dt>0);
  assert(_speed.size());
  return _min_speed;
}

double Trajectory::getMaxSpeed() const
{
  assert(_dt>0);
  assert(_speed.size());
  return _max_speed;
}

double Trajectory::getMinAcceleration() const
{
  assert(_dt>0);
  assert(_acceleration.size());
  return _min_acceleration;
}

double Trajectory::getMaxAcceleration() const
{
  assert(_dt>0);
  assert(_acceleration.size());
  return _max_acceleration;
}

double Trajectory::getMinJerk() const
{
  assert(_dt>0);
  assert(_jerk.size());
  return _min_jerk;
}

double Trajectory::getMaxJerk() const
{
  assert(_dt>0);
  assert(_jerk.size());
  return _max_jerk;
}

double Trajectory::getTotalSquaredJerk() const
{
  assert(_dt>0);
  double res = 0.0;
  for (int i=0; i<_jerk.size(); i++)
    res += _jerk[i] * _jerk[i];
  return res;
}

double Trajectory::getCost(double target_time, double target_speed, double max_speed, double max_acceleration, double max_jerk) const {
  double res = 0.0;
  double max_cost = std::numeric_limits<double>::max();

  if (_jerk.size()==0)
    return max_cost;

  // penalize going over the limits
//  if (fabs(getMaxJerk()) > max_jerk)
//    return max_cost;
//  if (fabs(getMaxAcceleration()) > max_acceleration)
//    return max_cost;
  if (getMaxSpeed() > max_speed)
    return max_cost;
  if (getMinSpeed() < 0.0)
    return max_cost;
//  if (target_time > getTotalT())
//    return max_cost;

  // time -> target
  double t = getTotalT() - target_time;
  res += fabs(t) / sqrt(1+t*t);
  // distance -> max
//  double d = getTotalDistance();
//  if (d>1e-6)
//    res += 1 / d;
  // speed -> target
  double v = getFinalSpeed() - target_speed;
  res += fabs(v) / sqrt(1+v*v);
  // average speed -> max
  double av = getTotalDistance() / getTotalT();
  res += 1 / av;
  // average square jerk -> min
  if (_jerk.size() > 0) {
    double aj2 = 0.0;
    for (int i = 0; i < _jerk.size(); i++)
      aj2 += _jerk[i] * _jerk[i];
    aj2 /= _jerk.size(); // average jerk
    aj2 /= max_jerk*max_jerk; // scale down using max jerk
    res += aj2;
  }

  return res;
}
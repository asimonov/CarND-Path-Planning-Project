//
// Created by Alexey Simonov on 05/08/2017.
//

#include "Trajectory.h"
#include <cassert>
#include "coordinate_utils.h"
#include <limits>
#include <fstream>

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

  _total_distance = 0.0;
  _min_speed = numeric_limits<double>::max();
  _max_speed = numeric_limits<double>::min();
  _min_acceleration = numeric_limits<double>::max();
  _max_acceleration = numeric_limits<double>::min();
  _min_jerk = numeric_limits<double>::max();
  _max_jerk = numeric_limits<double>::min();

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

void Trajectory::storeFinalFrenet(double s, double d)
{
  _final_s = s;
  _final_d = d;
}

std::vector<double> Trajectory::getFinalFrenet() const
{
  return {_final_s, _final_d};
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



double Trajectory::getCost(double target_time, double target_distance, double target_speed,
                           double max_speed, double max_acceleration, double max_jerk,
                           const Route& route) const
{
  double total_cost = 0.0;
  double MIN_COST = -1.0;
  double MAX_COST =  1.0;

  assert(_dt>0);
  // to avoid dealing with empty trajectories/div by zero
  if (_jerk.size()==0)
    return 100 * MIN_COST;

  _cost_dump_str += "COST COMPONENTS\n";

  // Penalize duration which is longer or shorter than the target duration
  double target_time_cost = logistic( fabs(getTotalT()-target_time) / target_time);
  double target_time_weight = 1.0;
  total_cost += target_time_weight * target_time_cost;
  _cost_dump_str += "target time cost: "+to_string(target_time_cost)+"\n";

  // Penalize distance which is longer or shorter than the target distance
  double target_distance_cost = logistic( fabs(getTotalDistance()-target_distance) / target_distance);
  double target_distance_weight = 1.0;
  total_cost += target_distance_weight * target_distance_cost;
  _cost_dump_str += "target distance cost: "+to_string(target_distance_cost)+"\n";

  // Penalize longer trajectories
  double total_time_cost = logistic( getTotalT() / 7.0);
  double total_time_weight = 1.0;
  total_cost += total_time_weight * total_time_cost;
  _cost_dump_str += "total time cost: "+to_string(total_time_cost)+"\n";


  // Penalise negative speeds
  double min_speed_cost = MIN_COST;
  if (_min_speed < 0.0)
    min_speed_cost = MAX_COST;
  double min_speed_weight = 1.0;
  total_cost += min_speed_weight * min_speed_cost;
  _cost_dump_str += "min speed cost: "+to_string(min_speed_cost)+"\n";

  // Penalise going over speed limit
  double speed_limit_cost = MIN_COST;
  if (_max_speed > max_speed)
    speed_limit_cost = MAX_COST;
  double speed_limit_weight = 1.0;
  total_cost += speed_limit_weight * speed_limit_cost;
  _cost_dump_str += "speed limit cost: "+to_string(speed_limit_cost)+"\n";

  // Reward higher average speeds.
  double avg_speed = getTotalDistance() / getTotalT();
  double avg_speed_cost = logistic( 2.0*(avg_speed) / max_speed);
  double avg_speed_weight = 1.0;
  total_cost += avg_speed_weight * avg_speed_cost;
  _cost_dump_str += "avg speed cost: "+to_string(avg_speed_cost)+" ("+to_string(avg_speed)+")\n";

  // Reward trajectories with final speed closer to target speed
  double final_speed = getFinalSpeed();
  double final_speed_cost = logistic(2.0*(target_speed - final_speed) / target_speed);
  double final_speed_weight = 1.0;
  total_cost += final_speed_weight * final_speed_cost;
  _cost_dump_str += "final speed cost: "+to_string(final_speed_cost)+" ("+to_string(final_speed)+")\n";


  // penalize trajectories with high max acceleration
  double max_acceleration_cost = MIN_COST;
  if (fabs(getMaxAcceleration()) > max_acceleration)
    max_acceleration_cost = MAX_COST;
  double max_acceleration_weight = 1.0;
  total_cost += max_acceleration_weight * max_acceleration_cost;
  _cost_dump_str += "max acceleration cost: "+to_string(max_acceleration_cost)+" ("+to_string(fabs(getMaxAcceleration()))+")\n";

  // penalize trajectories with high avg acceleration
  double total_acceleration = 0.0;
  for (int i=0;i<_acceleration.size(); i++)
    total_acceleration += fabs(_acceleration[i]*_dt);
  double acceleration_per_second = total_acceleration / getTotalT();
  double avg_acceleration_cost = logistic(acceleration_per_second / max_acceleration );
  double avg_acceleration_weight = 1.0;
  total_cost += avg_acceleration_weight * avg_acceleration_cost;
  _cost_dump_str += "avg acceleration cost: "+to_string(avg_acceleration_cost)+" ("+to_string(acceleration_per_second)+")\n";


  // penalize trajectories with high max jerk
  double max_jerk_cost = MIN_COST;
  if (fabs(getMaxJerk()) > max_jerk)
    max_jerk_cost = MAX_COST;
  double max_jerk_weight = 1.0;
  total_cost += max_jerk_weight * max_jerk_cost;
  _cost_dump_str += "max jerk cost: "+to_string(max_jerk_cost)+" ("+to_string(getMaxJerk())+")\n";

  // penalize trajectories with high avg jerk
  double total_jerk = 0.0;
  for (int i=0;i<_jerk.size(); i++)
    total_jerk += fabs(_jerk[i]*_dt);
  double jerk_per_second = total_jerk / getTotalT();
  double avg_jerk_cost = logistic(jerk_per_second / max_jerk );
  double avg_jerk_weight = 1.0;
  total_cost += avg_jerk_weight * avg_jerk_cost;
  _cost_dump_str += "avg jerk cost: "+to_string(avg_jerk_cost)+" ("+to_string(jerk_per_second)+")\n";



//  // Binary cost function which penalizes collisions.
//  double collision_cost = MIN_COST;
//  // Penalize getting close to other vehicles.
//  double buffer_cost = MIN_COST;
//  int n = sf.size();
//  const double check_dt = 0.1;
//  const double CAR_LENGTH = 5.0;
//  const double LANE_WIDTH = 4.0;
//  const double BUFFER_LENGTH = 10.0;
//  // loop over other cars
//  for (int i=0; i<n; i++)
//  {
//    const Car& c = sf.getCar(i);
//    double x0 = c.getX();
//    double y0 = c.getY();
//    double yaw = c.getYaw();
//    double v0 = c.getSpeed();
//    auto fr = route.get_frenet(x0, y0, yaw);
//    int step_size = floor(check_dt / _dt);
//    // loop over time
//    for (int j=1; j<getSize(); j+=step_size)
//    {
//      double s0 = fr[0];
//      double s1 = s0 + (j*_dt) * v0; // project other car location at new time
//      double d0 = fr[1];
//      double d1 = d0;
//      auto xy_other = route.get_XY(s1, d1);
//      double x_ego = _x_vals[i];
//      double y_ego = _y_vals[i];
//      auto fr_ego = route.get_frenet(x_ego, y_ego, _heading[i-1]);
//      double s_dist = s1 - fr_ego[0]; // distance to them. if they are in front, then positive
//      double d_dist = d1 - fr_ego[1];
//
//      // collision
//      if (s_dist>0 && s_dist <= CAR_LENGTH && fabs(d_dist) <= LANE_WIDTH)
//        collision_cost = MAX_COST;
//      // buffer
//      if (s_dist>0 && fabs(d_dist)<= LANE_WIDTH)
//        buffer_cost += logistic( s_dist / CAR_LENGTH);
//    }
//  }
//
//  double collision_weight = 1.0;
//  total_cost += collision_weight * collision_cost;
//  _cost_dump_str += "collision cost: "+to_string(collision_cost)+" \n";
//
//  double buffer_weight = 1.0;
//  total_cost += avg_jerk_weight * avg_jerk_cost;
//  _cost_dump_str += "buffer cost: "+to_string(buffer_cost)+" \n";



  _cost_dump_str += "TOTAL cost: "+to_string(total_cost)+"\n";
  return total_cost;
}


void Trajectory::dump_to_file(const std::string& filename) const
{
  ofstream f(filename.c_str(), ofstream::out);

  f<<"t,x,y"<<endl;
  int n = _x_vals.size();
  for (int i=0;i<n;i++)
  {
    f<<(i*_dt)<<" "<<_x_vals[i]<<" "<<_y_vals[i]<<endl;
  }

  f<<endl;

  f<<"dist"<<endl;
  n = _dist.size();
  for (int i=0;i<n;i++)
    f<<_dist[i]<<endl;

  f<<endl;

  f<<"speed"<<endl;
  n = _speed.size();
  for (int i=0;i<n;i++)
    f<<_speed[i]<<endl;

  f<<endl;

  f<<"acceleration"<<endl;
  n = _acceleration.size();
  for (int i=0;i<n;i++)
    f<<_acceleration[i]<<endl;

  f<<endl;

  f<<"jerk"<<endl;
  n = _jerk.size();
  for (int i=0;i<n;i++)
    f<<_jerk[i]<<endl;

  f<<endl;

  f<<"heading, degrees"<<endl;
  n = _heading.size();
  for (int i=0;i<n;i++)
    f<<rad2deg(_heading[i])<<endl;

  f<<endl;

  f<<"total time/distance: "<<getTotalT()<<" "<<_total_distance<<endl;
  f<<"min/max:"<<endl;
  f<<"speed: "<<_min_speed<<" "<<_max_speed<<endl;
  f<<"acceleration: "<<_min_acceleration<<" "<<_max_acceleration<<endl;
  f<<"jerk: "<<_min_jerk<<" "<<_max_jerk<<endl;

  f<<endl;

  f<<_cost_dump_str<<endl;

  f.close();
}

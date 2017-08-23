//
// Created by Alexey Simonov on 05/08/2017.
//

#include "Car.h"
#include <math.h>
#include <cassert>
#include "coordinate_utils.h"
#include <fstream>
#include <ios>

using namespace std;

Car::Car(int id,
         double x, double y, double yaw,
         double s, double d,
         int lane,
         double speed, double acceleration,
         double target_speed,
         double max_speed, double max_acceleration, double max_jerk
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
  assert(max_speed>0);
  _max_speed = max_speed;
  assert(max_acceleration>0);
  _max_acceleration = max_acceleration;
  assert(max_jerk>0);
  _max_jerk = max_jerk;

  _state = pair<Maneuvre ,int>(CONSTANT_SPEED, _lane); // default state
  _predictions_dt = -1.0;
}

Car Car::advance(double T) const
{
  Car advanced(*this); // use copy
  advanced._speed += _acceleration * T;
  advanced._s += _speed * T + 0.5 * _acceleration * T * T;
  return advanced;
}

// generate predictions. depends on the state
void Car::generate_predictions(double T, double dt)
{
  assert(dt>0);
  _predictions_dt = dt;
  _predictions.clear();

  Maneuvre m = _state.first;
  int l = _state.second;
  switch (m)
  {
    case CONSTANT_SPEED:
      for (double t=0; t<=T; t+=dt) {
        _predictions.push_back({_lane, _s + _speed * t});
      }
      break;
    case KEEP_LANE:
    case PREPARE_CHANGE_LANE:
      for (double t=0; t<=T; t+=dt) {
        double s = _s + _speed * t + 0.5 * _acceleration * t * t;
        _predictions.push_back({_lane, s});
      }
      break;
    case CHANGE_LANE:
      for (double t=0; t<=T; t+=dt) {
        double s = _s + _speed * t + 0.5 * _acceleration * t * t;
        if (t<=T/2.0)
          l = _lane; // assume lane change happens in the middle of time horizon
        _predictions.push_back({l, s});
      }
      break;
  }
}

double Car::maxAccelerationForLane(const std::vector<Car>& other_cars, double maneuvre_time) {
  double delta_v_til_target = _target_speed - _speed;
//  double t = maneuvre_time;
  double acc_til_target = delta_v_til_target / maneuvre_time; // can be negative
  double acc_sign = 1;
  if (acc_til_target<0) {
    acc_sign = -1;
  }
  double max_acc = acc_sign * min(_max_acceleration * 0.8, fabs(acc_til_target));

  //in_front = [v for (v_id, v) in predictions.items() if v[0]['lane'] == lane and v[0]['s'] > s]
  double leading_s_now = 1e+10;
  double leading_next_pos = 0.0;
  for (auto it = other_cars.begin(); it != other_cars.end(); it++)
    if (it->getLane() == _lane && it->getS() > _s) {
      // there is car in front
      if (it->getS() < leading_s_now) {
        leading_s_now = it->getS();
        leading_next_pos = it->advance(maneuvre_time).getS();
      }
    }

  if (leading_s_now < 1e+10 && acc_sign>0)
  {
    double my_next_pos = _s + _speed * maneuvre_time; // my position at current speed
    double separation_next = leading_next_pos - my_next_pos;
    double available_room = separation_next - 2*LENGTH; // add a buffer of 2 car lengths
    double balancing_acceleration = 2*available_room / (maneuvre_time*maneuvre_time); // can be negative
    if (balancing_acceleration<0) {
      acc_sign = -1;
      max_acc = acc_sign * min(_max_acceleration*0.8, fabs(balancing_acceleration));
    } else {
      max_acc = min(max_acc, balancing_acceleration);
    }
  }

  return max_acc;
}

void Car::setState(std::pair<Maneuvre, int>& state, const std::vector<Car>& other_cars, double maneuvre_time)
{
  _state = state;
  Maneuvre m = _state.first;
  int l = _state.second;
  assert(maneuvre_time>0.0);
  // realizing the state by adjusting speed/acceleration
  switch (m)
  {
    case CONSTANT_SPEED:
      _acceleration = 0.0;
      break;
    case KEEP_LANE:
      _acceleration = maxAccelerationForLane(other_cars, maneuvre_time);
      break;
    case CHANGE_LANE:
      _lane = l;
      _acceleration = maxAccelerationForLane(other_cars, maneuvre_time);
      break;
    case PREPARE_CHANGE_LANE:
      int old_lane = _lane;
      _lane = l;
      // what position we aim for, as of now, to merge into desired lane
      double best_target_s = _s;
      double best_target_v = _speed;
      // eliminate cars (from copy) until we are left with only cars in the target lane that are behind us
      vector<Car> cars_copy = other_cars;
      while (cars_copy.size())
      {
        if (cars_copy[cars_copy.size()-1].getLane() != _lane)
        {
          cars_copy.pop_back();
          continue;
        }
        // car in the lane we want to merge into.
        Car c = cars_copy[cars_copy.size()-1];
        // we are not interested in cars in front as we are not considering speeding up.
        // so we only look at cars behind our position+buffer
        if (c.getS() > _s + 2*LENGTH)
        {
          cars_copy.pop_back();
          continue;
        }
        break;
      }
      if (cars_copy.size())
      {
        // find the closest car behind (descending sort on distance)
        // TODO deal with S wrapping around to zero around the track, in which case it may be different car that we need
        sort(cars_copy.begin(), cars_copy.end(), [](const Car& x, const Car& y)->bool{return x.getS() > y.getS();});
        Car nearest_behind = cars_copy[0];
        //Car nearest_behind_target = nearest_behind.advance(maneuvre_time);

        //double delta_v = nearest_behind.getSpeed() - _speed; // new speed is _speed plus delta
        double delta_s = (nearest_behind.getS() - 2*LENGTH) - _s;// wanna end up behind that car
        // calculate [de]acceleration to end up behind that car at specified time horizon
        double a = -2.0 * (delta_s + _speed*maneuvre_time) / (maneuvre_time * maneuvre_time);
        if (a<0)
          _acceleration = -min(_max_acceleration*0.8,fabs(a));
        else
          _acceleration = min(_max_acceleration*0.8,a);
      }
      else
      {  // no potential obstacles in the target lane, pick best acceleration
        _acceleration = maxAccelerationForLane(other_cars, maneuvre_time);
      }

      _lane = old_lane;
      break;
  }

}

double Car::get_target_time() const
{
  assert(_predictions_dt>0);
  assert(_predictions.size());
  return (_predictions.size()-1)*_predictions_dt;
}

int    Car::get_target_lane() const
{
  int res = _lane;
  if (_state.first == CHANGE_LANE)
    res = _state.second;
  return res;
}


// calculates cost function of internal trajectory given trajectories of other cars
double Car::calculate_cost(const std::vector<Car>& other_cars)
{
  double total_cost = 0.0;

  // cost boundaries for each instance of priced event
  // all costs are between MIN and MAX, before priority levels are applied
  const double MIN_COST   = +0.;
  const double MAX_COST   = +1.;
  // priority levels for different costs
  const double COLLISION  = 100;
  const double DANGER     = 50;
  //const double REACH_GOAL = 1e+5;
  const double COMFORT    = 30;
  const double EFFICIENCY = 15;

  double collision_cost = MIN_COST;
  double buffer_cost = MIN_COST;
  int lane_change_count = 0;
  assert(_predictions_dt>0);
  assert(_predictions.size());
  double maneuvre_time = (_predictions.size()-1) * _predictions_dt;
  double maneuvre_distance = (_predictions[_predictions.size()].second - _predictions[0].second);

  for (auto it=other_cars.begin(); it!=other_cars.end(); it++)
  {
    assert(_predictions_dt = it->_predictions_dt); // make sure our time steps are same for predictions
    int ego_lane = _lane; // initial lane
    double this_car_buffer_cost = MIN_COST;
    for (int i=0; i<min(_predictions.size(), it->_predictions.size()); i++)
    {
      if (ego_lane != _predictions[i].first)
        lane_change_count += 1;
      ego_lane = _predictions[i].first; // possibly changed lane
      int other_lane = it->_predictions[i].first;
      if (_s > it->getS()+LENGTH) {
        // only take into consideration cars which were initially in front.
        // If they are behind, it is their problem to keep distance and avoid collisions
        break;
      }
      if (ego_lane == other_lane)
      {
        double ego_s = _predictions[i].second;
        double other_s = it->_predictions[i].second;
        // TODO take care of S wrapping around zero
        if (fabs(ego_s - other_s) < LENGTH) {
          collision_cost += MAX_COST;
          break;
        }
        const int NUM_BUFFER_LENGTHS = 10;
        if (fabs(ego_s - other_s) < NUM_BUFFER_LENGTHS*LENGTH)
        {
          // only take buffer into consideration if we are behind a car initially.
          // they they are behind, it is their problem to keep distance.
          // Expression in logistic function should be between 0 and 5.
          double bcost = MAX_COST - logistic( fabs(ego_s - other_s) / (2*NUM_BUFFER_LENGTHS*LENGTH) );
          if (bcost > this_car_buffer_cost)
            this_car_buffer_cost = bcost;
        }
      }
    }
    if (this_car_buffer_cost > buffer_cost)
      buffer_cost = this_car_buffer_cost;
  }

  //  collision_cost
  total_cost += COLLISION * collision_cost;

  //  buffer_cost,
  total_cost += DANGER * buffer_cost;

  //  change_lane_cost. drops with length of the maneuvre, if we did change lane
  double lane_change_cost = MIN_COST;
  if (lane_change_count)
  {
    // lane change should not take more than 3 seconds
    if (maneuvre_time > 4.5)
      lane_change_cost = MAX_COST;
    else
      lane_change_cost = logistic(15 * (lane_change_count*WIDTH) / maneuvre_distance);
  }
  total_cost += COMFORT * lane_change_cost;


  double max_speed = -1e+10;
  double max_acceleration = max_speed;
  double prev_s = _s;
  double max_s = _s;
  double max_backwards_distance = 0.0;
  double prev_v = _speed;
  bool went_backwards = false;
  for (int i=1; i<_predictions.size(); i++)
  {
    double next_s = _predictions[i].second;
    if (next_s > max_s)
      max_s = next_s;
    if (next_s < max_s) {
      went_backwards = true;
      if (max_s - next_s > max_backwards_distance)
        max_backwards_distance = max_s - next_s;
    }
    // TODO take care of S wrapping around zero
    double v = (next_s - prev_s) / _predictions_dt;
    if (v > max_speed)
      max_speed = v;
    double a = (v - prev_v) / _predictions_dt;
    if (a> max_acceleration)
      max_acceleration = a;
    prev_s = next_s;
    prev_v = v;
  }

  // speed limit cost
  double speed_limit_cost = 0.0;
  if (max_speed > _target_speed && max_speed < _max_speed)
    speed_limit_cost = (max_speed - _target_speed) * MAX_COST / (_max_speed - _target_speed);
  if (max_speed > _max_speed)
    speed_limit_cost = MAX_COST;
  total_cost += DANGER * speed_limit_cost;

  // max acceleration cost
  double max_acceleration_cost = 0.0;
  if (max_acceleration > 0.8 * _max_acceleration && max_acceleration < _max_acceleration)
    max_acceleration_cost = (max_acceleration - 0.8 * _max_acceleration) * MAX_COST / (_max_acceleration - 0.8*_max_acceleration);
  total_cost += COMFORT * max_acceleration_cost;

  //  inefficiency_cost, maximize average speed
  // TODO handle s wrapping around the track to zero
  double avg_speed = maneuvre_distance / maneuvre_time;
  double avg_speed_cost = 1.0 - logistic(avg_speed / _target_speed);
  total_cost += EFFICIENCY * avg_speed_cost;

  // disallow going backwards
  double go_backwards_cost = MIN_COST;
  if (went_backwards)
    go_backwards_cost = logistic(max_backwards_distance);
  total_cost += COMFORT * go_backwards_cost;

  return total_cost;
}

void Car::dumpToStream(const std::string& filename) const
{
  ofstream out_stream(filename.c_str(), ios_base::app);
  out_stream << "id   : "<< _id << endl;
  out_stream << "x,y  : "<< _x << " " << _y << endl;
  out_stream << "yaw  : "<< _yaw << endl;
  out_stream << "s,d  : "<< _s << " " << _d << endl;
  out_stream << "lane : "<< _lane << endl;
  out_stream << "speed: "<< _speed << endl;
  out_stream << "acc  : "<< _acceleration << endl;
  out_stream << "maneuvre : "<< _state.first << endl;
  out_stream << "target lane : "<< _state.second << endl;
  out_stream << endl;
  out_stream.close();
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


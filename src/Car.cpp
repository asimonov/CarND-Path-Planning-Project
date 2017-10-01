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
        // assume lane change happens in the middle of time horizon
        if (t<=T/2.0)
          _predictions.push_back({_lane, s});
        else
          _predictions.push_back({l, s});
      }
      break;
  }
}

double Car::maxAccelerationForLane(const std::vector<Car>& other_cars, double maneuvre_time) {
  double delta_v_til_target = _target_speed - _speed;
  double acc_til_target = delta_v_til_target / maneuvre_time; // can be negative
  double acc_sign = 1;
  if (acc_til_target<0) {
    acc_sign = -1;
  }
  // this is acceleration to get to target speed in maneuvre_time. with no obstacles
  double max_acc = acc_sign * min(_max_acceleration * 0.8, fabs(acc_til_target));

  // find nearest car in front that we may need to follow
  double leading_s_now = 1e+10;
  Car car(*this);
  for (auto it = other_cars.begin(); it != other_cars.end(); it++)
    if (it->getLane() == _lane && it->getS() > _s) {
      // there is car in front
      if (it->getS() < leading_s_now) {
        leading_s_now = it->getS();
        car = *it; // copy
      }
    }

  // adjust our acceleration based on leading vehicle
  int NUM_LENGTHS_BEHIND_TARGET = 3;
  if (leading_s_now < 1e+10)// && acc_sign>0)
  {
    double delta_s = (car.getS() - NUM_LENGTHS_BEHIND_TARGET*LENGTH) - _s;
    delta_s += (car.getSpeed() - _speed)*maneuvre_time;
    // calculate de/acceleration to end up behind that car at specified time horizon
    double a = 2.0 * delta_s / (maneuvre_time * maneuvre_time);
    if (a<max_acc)
      max_acc = a;
    if (max_acc<0)
      max_acc = max(-_max_acceleration*0.8,max_acc);
    else
      max_acc = min(_max_acceleration*0.8,max_acc);
  }

  return max_acc;
}

void Car::setState(std::pair<Maneuvre, int>& state, const std::vector<Car>& other_cars, double maneuvre_time)
{
  // sets _state, _lane, _acceleration
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
      int NUM_LENGTHS_BEHIND_TARGET = 3;
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
        if (c.getS() > _s + NUM_LENGTHS_BEHIND_TARGET*LENGTH)
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

        //double delta_v = nearest_behind.getSpeed() - _speed; // new speed is _speed plus delta
        double delta_s = (nearest_behind.getS() - NUM_LENGTHS_BEHIND_TARGET*LENGTH) - _s; // negative distance
        delta_s += (nearest_behind.getSpeed() - _speed)*maneuvre_time;
        // calculate [de]acceleration to end up behind that car at specified time horizon
        double a = 2.0 * delta_s / (maneuvre_time * maneuvre_time);
        if (a<0)
          _acceleration = max(-_max_acceleration*0.8,a);
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
  return (_predictions.size())*_predictions_dt;
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
  const double MIN_COST   = 0.0;
  const double MAX_COST   = 1.0;
  // priority levels for different costs
  const double COLLISION  = 100;
  const double DANGER     = 50;
  //const double REACH_GOAL = 1e+5;
  const double COMFORT    = 20;
  const double EFFICIENCY = 30;

  double collision_cost = MIN_COST;
  double buffer_cost = MIN_COST;
  assert(_predictions_dt>0);
  assert(_predictions.size());
  double maneuvre_time = (_predictions.size()-1) * _predictions_dt;
  double maneuvre_distance = (_predictions[_predictions.size()].second - _predictions[0].second);

  for (auto other_car=other_cars.begin(); other_car!=other_cars.end(); other_car++)
  {
    assert(_predictions_dt = other_car->_predictions_dt); // make sure our time steps are same for predictions
    double this_car_buffer_cost = MIN_COST;
    double initial_distance = _s - (other_car->getS()+LENGTH);
    for (int i=0; i<min(_predictions.size(), other_car->_predictions.size()); i++)
    {
      int other_lane = other_car->_predictions[i].first;
      if (_predictions[i].first == other_lane)
      {
        double ego_s = _predictions[i].second;
        double other_s = other_car->_predictions[i].second;
        double abs_distance = fabs(ego_s - other_s);

        // taking care of S wrapping around zero in an ugly manner. we should ask Route about max s, but we do not have
        // it here. so hardcoding for this particular task. ugly ugly...
        if (abs_distance > 6945.554)
          abs_distance -= 6945.554;

        const int NUM_BUFFER_LENGTHS = 4;

        if (abs_distance < LENGTH) {
          if (initial_distance>0)
            // we were ahead
            if (initial_distance < NUM_BUFFER_LENGTHS*LENGTH && _state.first == CHANGE_LANE)
              // we were slightly ahead but cut him up. should not really do this...
              collision_cost += MAX_COST / 10;
            else
              ; // not our problem.
          else
            // we were behind and crashed into other car. baaaaad
            collision_cost += MAX_COST;
          break;
        }

        if (initial_distance < 0 && abs_distance < 2*NUM_BUFFER_LENGTHS*LENGTH)
        {
          // only take buffer into consideration if we are behind a car initially.
          // they they are behind, other_car is their problem to keep distance.
          // Expression in logistic function should be between 0 and 5.
          double bcost = MAX_COST - logistic( abs_distance / (NUM_BUFFER_LENGTHS*LENGTH) );
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
  if (_state.first == CHANGE_LANE)
  {
    if (maneuvre_time > 3.0 || _speed < 3.0)
      // lane change should not take more than 3 seconds
      // and we should not try to change lane until we speed up somewhat
      lane_change_cost = MAX_COST;
    else {
      // but lane change should not be very rushed, should be extended over some time
      // normal acceleration at constant speed is proportional to square speed
      // and inversely proportional to curvature, which is roughly maneuvre distance
      double avg_speed = maneuvre_distance/maneuvre_time;
      lane_change_cost = logistic(pow(avg_speed/4., 2) / (maneuvre_distance/2.));
    }
  }
  else if (_state.first == PREPARE_CHANGE_LANE)
  {
    // it is not full blown lane change, but it involves us slowing down potentially
    // so we discourage this.
    // will also result in less lane changes
    lane_change_cost = MAX_COST / 2;
  }
  total_cost += COMFORT * lane_change_cost;

  // speed related costs
  double start_speed = _speed;
  double end_speed = start_speed + _acceleration * maneuvre_time;
  double max_speed = max(start_speed, end_speed);
  double min_speed = min(start_speed, end_speed);

  // speed limit cost
  double speed_limit_cost = 0.0;
  if (max_speed > _max_speed)
    speed_limit_cost = MAX_COST;
  else if (max_speed > _target_speed && max_speed < _max_speed)
    speed_limit_cost = (max_speed - _target_speed) * MAX_COST / (_max_speed - _target_speed);
  total_cost += DANGER * speed_limit_cost;

  // negative speed cost
  double negative_speed_cost = 0.0;
  if (min_speed < 0.0)
    negative_speed_cost = MAX_COST;
  total_cost += EFFICIENCY * negative_speed_cost;

  // max acceleration cost
  double max_acceleration_cost = 0.0;
  if (_acceleration > _max_acceleration)
    max_acceleration_cost = MAX_COST;
  else if (_acceleration > 0.8 * _max_acceleration && _acceleration < _max_acceleration)
    max_acceleration_cost = (_acceleration - 0.8 * _max_acceleration) * MAX_COST / (_max_acceleration - 0.8*_max_acceleration);
  total_cost += COMFORT * max_acceleration_cost;

  // maximize average speed
  // TODO handle s wrapping around the track to zero
  double avg_speed = maneuvre_distance / maneuvre_time;
  double SPEED_SENSITIVITY = 5;
  double avg_speed_cost = 1.0 - logistic((avg_speed - _target_speed)/SPEED_SENSITIVITY);
  total_cost += EFFICIENCY * avg_speed_cost;

  // penalise deceleration
  double neg_acceleration_cost = 0.0;
  if (_acceleration < 0)
    neg_acceleration_cost = logistic(fabs(_acceleration) / (_max_acceleration/2.0));
  total_cost += EFFICIENCY * neg_acceleration_cost;

  // penalise big acceleration/deceleration
  double acceleration_cost = logistic(pow(_acceleration / (_max_acceleration/2.5), 2));
  total_cost += COMFORT * acceleration_cost;

  // now if all is clear then for efficiency we want to keep lane rather than prepare to change it
  if (_state.first == PREPARE_CHANGE_LANE)
    total_cost *= 1.05; // make it 5 percent more costly to PREPARE_CHANGE_LANE to favour KEEP_LANE provided all else is equal
  if (_state.first == CHANGE_LANE)
    total_cost *= 1.1; // make it 10 percent more costly to CHANGE_LANE to favour KEEP_LANE provided all else is equal

  return total_cost;
}

void Car::dumpToStream(const std::string& filename) const
{
  ofstream out_stream(filename.c_str(), ios_base::app);
  out_stream << "lane : "<< _lane << endl;
  out_stream << "id   : "<< _id << endl;
  out_stream << "x,y  : "<< _x << " " << _y << endl;
  out_stream << "yaw  : "<< _yaw << endl;
  out_stream << "s,d  : "<< _s << " " << _d << endl;
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


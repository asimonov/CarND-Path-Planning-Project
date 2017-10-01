//
// Created by Alexey Simonov on 14/08/2017.
//

#include "BehaviourPlanner.h"
#include <map>
#include <iostream>
#include <cassert>
#include <cmath>
#include "coordinate_utils.h"

using namespace std;

BehaviourPlanner::BehaviourPlanner(int num_lanes,
                                   double lane_width,
                                   Car ego,
                                   std::vector<Car>& other_cars,
                                   double s_wrap_length) : _ego(ego), _other_cars(other_cars), _dt(0.1)
{
  _num_lanes = num_lanes;
  _s_wrap_length = s_wrap_length;
}

Car BehaviourPlanner::plan(double T_horizon)
{
  /*
  Updates the "state" of the ego vehicle based on 'best' maneuvre to take:

  "KL" - Keep Lane
   - The vehicle will attempt to drive its target speed, unless there is
     traffic in front of it, in which case it will slow down.

  "LC" - Lane Change (Left / Right -- see target_lane)
   - The vehicle will change lanes and then follow longitudinal
     behavior for the "KL" state in the new lane.

  "PLC" - Prepare for Lane Change (Left / Right -- see target_lane)
   - The vehicle will find the nearest vehicle in the adjacent lane which is
     BEHIND itself and will adjust speed to try to get behind that vehicle.
  */

  // generate predictions
  for(auto it = _other_cars.begin(); it!=_other_cars.end(); it++)
    it->generate_predictions(T_horizon, _dt);

  // current state
  pair<Maneuvre, int> current_state = _ego.getState();

  // the resulting object. by the end should have planned state, lane, speed, acceleration etc
  Car egoPlan = _ego; // copy from existing one

  // possible new states
  vector<pair<Maneuvre, int>> possible_states;
  if (current_state.first == KEEP_LANE || current_state.first == CONSTANT_SPEED) {
    // possible states from here: KL, LC, PLC
    int lane = current_state.second;
    possible_states.push_back(pair<Maneuvre, int>(KEEP_LANE, lane));
    if (lane > 0) // can change left
    {
      possible_states.push_back(pair<Maneuvre, int>(CHANGE_LANE, lane - 1));
      possible_states.push_back(pair<Maneuvre, int>(PREPARE_CHANGE_LANE, lane - 1));
    }
    if (lane < _num_lanes - 1) // can change right
    {
      possible_states.push_back(pair<Maneuvre, int>(CHANGE_LANE, lane + 1));
      possible_states.push_back(pair<Maneuvre, int>(PREPARE_CHANGE_LANE, lane + 1));
    }
  }
  else if (current_state.first == PREPARE_CHANGE_LANE) {
    // possible states from here: KL, LC, PLC
    int current_lane = egoPlan.getLane();
    int target_lane = current_state.second;
    possible_states.push_back(pair<Maneuvre, int>(KEEP_LANE, current_lane));
    if (current_lane > 0) // can change left
    {
      possible_states.push_back(pair<Maneuvre, int>(CHANGE_LANE, current_lane - 1));
      possible_states.push_back(pair<Maneuvre, int>(PREPARE_CHANGE_LANE, current_lane - 1));
    }
    if (current_lane < _num_lanes-1) // can change right
    {
      possible_states.push_back(pair<Maneuvre, int>(CHANGE_LANE, current_lane + 1));
      possible_states.push_back(pair<Maneuvre, int>(PREPARE_CHANGE_LANE, current_lane + 1));
    }
  } else if (current_state.first == CHANGE_LANE) {
    // possible states from here: LC, KL
    int target_lane = current_state.second;
    possible_states.push_back(pair<Maneuvre, int>(KEEP_LANE, target_lane));
    possible_states.push_back(pair<Maneuvre, int>(CHANGE_LANE, target_lane));
  }

  // find the state with smallest cost
  pair<Maneuvre, int> best_state;
  double best_cost = 1e+10;
  double best_t = 0.0;
  cout << "BP: start" << endl;
  for (auto state : possible_states)
  {
    cout << "BP: try state " << state.first << " " << state.second  << endl;
    // try several time horizons with each state, then choose based on cost
    for (double t=1.0; t<=T_horizon; t+=1.0) {
      cout << "BP: try time " << t << endl;
      Car egoCopy = egoPlan;
      egoCopy.setState(state,_other_cars,t);
      egoCopy.generate_predictions(t, _dt); // this also sets time horizon for ego to chose
      //double cost = egoCopy.calculate_cost(_other_cars);
      double cost = calculate_cost(egoCopy);
      cout << "BP: cost " << cost << ", acceleration " << egoCopy.getAcceleration() << " lane " << egoCopy.get_target_lane() <<  endl;
      if (cost < best_cost) {
        cout << "BP: *** " <<  endl;
        best_cost = cost;
        best_state = state;
        best_t = t;
      }
    }
  }
  egoPlan.setState(best_state,_other_cars,best_t);
  egoPlan.generate_predictions(best_t, _dt);

  return egoPlan;
}

double BehaviourPlanner::cyclic_dist(double s)
{
  double res = s;
  int cnt = 0;
  while (res>_s_wrap_length) {
    res -= _s_wrap_length;
    assert(cnt++ < 10);
  }
  return res;
}

double BehaviourPlanner::calculate_cost(const Car& ego) {
  // we use _other_cars expecting them to have right state/predictions
  double total_cost = 0.0;

  // cost boundaries for each instance of priced event
  // all costs are between MIN and MAX, before priority levels are applied
  const double MIN_COST   = 0.0;
  const double MAX_COST   = 1.0;
  // priority levels for different costs
  const double COLLISION  = 100;
  const double DANGER     = 50;
  //const double REACH_GOAL = 1e+5;
  const double COMFORT    = 30;
  const double EFFICIENCY = 20;

  double collision_cost = MIN_COST;
  double buffer_cost = MIN_COST;

  const predictions_type& ego_predictions = ego.get_predictions();
  assert(ego_predictions.size());
  double maneuvre_time = ego.get_target_time();
  double maneuvre_distance = cyclic_dist(ego_predictions[ego_predictions.size()-1].second - ego_predictions[0].second);

  for (auto other_car=_other_cars.begin(); other_car!=_other_cars.end(); other_car++)
  {
    double this_car_buffer_cost = MIN_COST;
    double initial_distance = cyclic_dist( ego.getS() - (other_car->getS() + ego.getLength()) );
    const predictions_type& other_predictions = other_car->get_predictions();
    for (int i=0; i<min(ego_predictions.size(), other_predictions.size()); i++)
    {
      int other_lane = other_predictions[i].first;
      if (ego_predictions[i].first == other_lane)
      {
        double ego_s = ego_predictions[i].second;
        double other_s = other_predictions[i].second;
        double abs_distance = cyclic_dist(fabs(ego_s - other_s));

        const int NUM_BUFFER_LENGTHS = 4;

        if (abs_distance < ego.getLength()) {
          if (initial_distance>0)
            // we were ahead
            if (initial_distance < NUM_BUFFER_LENGTHS*ego.getLength() && ego.getState().first == CHANGE_LANE)
              // we were slightly ahead but cut him up. should not really do this...
              collision_cost += MAX_COST / 10;
            else
              ; // not our problem.
          else
            // we were behind and crashed into other car. baaaaad
            collision_cost += MAX_COST;
          break;
        }

        if (initial_distance < 0 && abs_distance < 2*NUM_BUFFER_LENGTHS*ego.getLength())
        {
          // only take buffer into consideration if we are behind a car initially.
          // they they are behind, other_car is their problem to keep distance.
          // Expression in logistic function should be between 0 and 5.
          double bcost = MAX_COST - logistic( abs_distance / (NUM_BUFFER_LENGTHS*ego.getLength()) );
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
  if (ego.getState().first == CHANGE_LANE)
  {
    if (maneuvre_time > 3.0 || ego.getSpeed() < 3.0)
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
  else if (ego.getState().first == PREPARE_CHANGE_LANE)
  {
    // it is not full blown lane change, but it involves us slowing down potentially
    // so we discourage this.
    // will also result in less lane changes
    lane_change_cost = MAX_COST / 2;
  }
  total_cost += COMFORT * lane_change_cost;

  // speed related costs
  double start_speed = ego.getSpeed();
  double end_speed = start_speed + ego.getAcceleration() * maneuvre_time;
  double max_speed = max(start_speed, end_speed);
  double min_speed = min(start_speed, end_speed);

  // speed limit cost
  double speed_limit_cost = 0.0;
  if (max_speed > ego.getMaxSpeed())
    speed_limit_cost = MAX_COST;
  else if (max_speed > ego.getTargetSpeed() && max_speed < ego.getMaxSpeed())
    speed_limit_cost = (max_speed - ego.getTargetSpeed()) * MAX_COST / (ego.getMaxSpeed() - ego.getTargetSpeed());
  total_cost += DANGER * speed_limit_cost;

  // penalise negative speed
  double negative_speed_cost = 0.0;
  if (min_speed < 0.0)
    negative_speed_cost = MAX_COST;
  total_cost += EFFICIENCY * negative_speed_cost;

  // penalise max acceleration
  double max_acceleration_cost = 0.0;
  const double SAFE_ACCELERATION_FACTOR = 0.8;
  if (ego.getAcceleration() > ego.getMaxAcceleration())
    max_acceleration_cost = MAX_COST;
  else if (ego.getAcceleration() > SAFE_ACCELERATION_FACTOR * ego.getMaxAcceleration() && ego.getAcceleration() < ego.getMaxAcceleration())
    max_acceleration_cost = (ego.getAcceleration() - SAFE_ACCELERATION_FACTOR * ego.getMaxAcceleration()) * MAX_COST / (ego.getMaxAcceleration() - SAFE_ACCELERATION_FACTOR*ego.getMaxAcceleration());
  total_cost += COMFORT * max_acceleration_cost;

  // maximize average speed
  // TODO handle s wrapping around the track to zero
  double avg_speed = maneuvre_distance / maneuvre_time;
  double SPEED_SENSITIVITY = 5;
  double avg_speed_cost = 1.0 - logistic((avg_speed - ego.getTargetSpeed())/SPEED_SENSITIVITY);
  total_cost += EFFICIENCY * avg_speed_cost;

  // penalise deceleration
//  double neg_acceleration_cost = 0.0;
//  if (ego.getAcceleration() < 0)
//    neg_acceleration_cost = logistic(fabs(ego.getAcceleration()) / (ego.getMaxAcceleration()/2.0));
//  total_cost += EFFICIENCY * neg_acceleration_cost;

  // penalise big acceleration/deceleration
  double acceleration_cost = logistic(pow(ego.getAcceleration() / (ego.getMaxAcceleration()/2.5), 2));
  total_cost += COMFORT * acceleration_cost;

  // penalise longer maneuvers
  double maneuvre_time_cost = logistic(maneuvre_time / 3.0);
  total_cost += EFFICIENCY * maneuvre_time_cost;

  // now if all is clear then for efficiency we want to keep lane rather than prepare to change it
  if (ego.getState().first == PREPARE_CHANGE_LANE)
    total_cost *= 1.05; // make it 5 percent more costly to PREPARE_CHANGE_LANE to favour KEEP_LANE provided all else is equal
  if (ego.getState().first == CHANGE_LANE)
    total_cost *= 1.1; // make it 10 percent more costly to CHANGE_LANE to favour KEEP_LANE provided all else is equal

  return total_cost;
}

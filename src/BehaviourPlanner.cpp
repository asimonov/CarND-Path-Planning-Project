//
// Created by Alexey Simonov on 14/08/2017.
//

#include "BehaviourPlanner.h"
#include <map>

using namespace std;

BehaviourPlanner::BehaviourPlanner(int num_lanes, double lane_width, Car ego, std::vector<Car> other_cars) : _ego(ego)
{
  _num_lanes = num_lanes;
  _other_cars = other_cars;
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

  const double dt = 0.1; // when predicting, what discretisation frequency to use?
  // generate predictions
  for(auto it = _other_cars.begin(); it!=_other_cars.end(); it++)
    it->generate_predictions(T_horizon, dt);

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
  for (auto state : possible_states)
  {
    // try several time horizons with each state, then choose based on cost
    for (double t=1.0; t<=T_horizon; t+=1.0) {
      Car egoCopy = egoPlan;
      egoCopy.setState(state,_other_cars,t);
      egoCopy.generate_predictions(t, dt); // this also sets time horizon for ego to chose
      double cost = egoCopy.calculate_cost(_other_cars);
      if (cost < best_cost) {
        best_cost = cost;
        best_state = state;
        best_t = t;
      }
    }
  }
  egoPlan.setState(best_state,_other_cars,best_t);
  egoPlan.generate_predictions(best_t, dt);
  double cost = egoPlan.calculate_cost(_other_cars);

  return egoPlan;
}

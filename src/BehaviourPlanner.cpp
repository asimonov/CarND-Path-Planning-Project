//
// Created by Alexey Simonov on 14/08/2017.
//

#include "BehaviourPlanner.h"
#include <map>

using namespace std;

BehaviourPlanner::BehaviourPlanner(int num_lanes, double lane_width, Car ego, std::vector<Car> other_cars) : _ego(ego)
{
  _num_lanes = num_lanes;
  _lane_width = lane_width;
  _other_cars = other_cars;
}

Car BehaviourPlanner::plan(double T, double max_speed, double max_acceleration, double max_jerk)
{
  // the resulting object. by the end should have planned state, lane, speed, acceleration etc
  Car egoPlan = _ego; // copy from existing one

  const double dt = 0.1; // when predicting, what discretisation frequency to use?
  // generate predictions
  egoPlan.generate_predictions(T, dt);
  for(auto it = _other_cars.begin(); it!=_other_cars.end(); it++)
    it->generate_predictions(T, dt);

  /*
  Updates the "state" of the vehicle by assigning one of the
  following values to 'self.state':

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

  // current state
  string current_state = _ego.getState();
  int    current_lane = _ego.getLane();

  // new state
  string new_state = "KL";
  int    new_lane = current_lane;

  // find best new state
  if        (current_state.compare("KL") == 0) {
    // possible states from here: KL, LC, PLC
  } else if (current_state.compare("PLC") == 0) {
    // possible states from here: KL, LC, PLC
  } else if (current_state.compare("LC") == 0) {
    // possible states from here: LC, KL
  }

  //vector<int> cars_in_front;
  for(auto it = _other_cars.begin(); it!=_other_cars.end(); it++)
    ;//it->generate_predictions(T, dt);

  // set new state
  egoPlan.setState(new_state);
  egoPlan.setTargetLane(new_lane);

  /*
  Given new state, realize it by adjusting goal time, position, speed, acceleration
  */
  if (new_state.compare("KL") == 0) {
    // realize Keep Lane
    egoPlan.setAcceleration(0.0);
  } else if (new_state.compare("LC") == 0) {
    //realize_lane_change(predictions, "L");
  } else if (new_state.compare("PLC") == 0) {
    //realize_prep_lane_change(predictions, "L");
  }

  return egoPlan;
}

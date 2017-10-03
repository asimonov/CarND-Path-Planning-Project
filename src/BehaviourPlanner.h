//
// Created by Alexey Simonov on 14/08/2017.
//

#ifndef PATH_PLANNING_BEHAVIOURPLANNER_H
#define PATH_PLANNING_BEHAVIOURPLANNER_H

#include "Car.h"
#include <vector>

// Behaviour planner for highway driving.
// Assumes we have highway with a number of lanes
// and all cars move along them in same direction
// with various speeds.


class BehaviourPlanner {
public:
    BehaviourPlanner(int num_lanes, double lane_width, Car ego, std::vector<Car>& other_cars, double s_wrap_length=1e+10);

    // Plans behavior of ego vehicle for up to time horizon T.
    // Returns state of ego vehicle representing the plan (target state, lane, distance, speed, acceleration)
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
    Car plan(double T);

    // calculates cost of the planned action (embedded in ego). Uses stored other_cars
    double calculate_cost(const Car& ego);

private:
    BehaviourPlanner();
    double cyclic_dist(double s);

    int               _num_lanes;
    std::vector<Car>&  _other_cars;
    Car               _ego;
    const double      _dt; // when predicting, what discretisation frequency to use?
    double            _s_wrap_length;
};


#endif //PATH_PLANNING_BEHAVIOURPLANNER_H

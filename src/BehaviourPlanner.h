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
// with various speeds/accelerations.


class BehaviourPlanner {
public:
    BehaviourPlanner(int num_lanes, double lane_width, Car ego, std::vector<Car> other_cars);

    // Plans behavior of ego vehicle for up to time horizon T.
    // Returns state of ego vehicle representing the plan (target state, lane, distance, speed, acceleration)
    Car plan(double T);
private:
    BehaviourPlanner();

    int               _num_lanes;
    //double            _lane_width;
    std::vector<Car>  _other_cars;
    Car               _ego;
};


#endif //PATH_PLANNING_BEHAVIOURPLANNER_H

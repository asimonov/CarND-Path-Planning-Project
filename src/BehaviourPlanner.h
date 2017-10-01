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
    BehaviourPlanner(int num_lanes, double lane_width, Car ego, std::vector<Car>& other_cars, double s_wrap_length=1e+10);

    // Plans behavior of ego vehicle for up to time horizon T.
    // Returns state of ego vehicle representing the plan (target state, lane, distance, speed, acceleration)
    Car plan(double T);
    double calculate_cost(const Car& ego);
private:
    BehaviourPlanner();
    double cyclic_dist(double s);

    int               _num_lanes;
    //double            _lane_width;
    std::vector<Car>&  _other_cars;
    Car               _ego;
    const double      _dt; // when predicting, what discretisation frequency to use?
    double            _s_wrap_length;
};


#endif //PATH_PLANNING_BEHAVIOURPLANNER_H

//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "Route.h"
#include "Car.h"
#include "Trajectory.h"


class JMTPlanner
{
    // extends trajectory, i.e. set of (x,y) coordinates spaced at some time interval
    // takes:
    //   car (position, heading, velocity),
    //   existing trajectory to be extended (can be empty),
    //   route to follow (as road centerline coordinates, also converts from frenet to XY and vice versa),
    //   planning time horizon planning_time,
    //   lane width (to get d from lane number)
    //   desired speed at the end of planning horizon
    //   max speed, acceleration and jerk,
public:
    virtual Trajectory extendTrajectory(const Car& car,
                                        const Trajectory& trajectory,
                                        const Route& route,
                                        double planning_time,
                                        double lane_width,
                                        double target_speed,
                                        double max_speed,
                                        double max_acceleration,
                                        double max_jerk);

private:
};

#endif //PATH_PLANNING_PATHPLANNER_H

//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "Route.h"
#include "Car.h"
#include "Trajectory.h"
#include "SensorFusion.h"


class PathPlanner {
public:
    // extends trajectory, i.e. set of (x,y) coordinates spaced at some time interval
    // takes:
    //   car (with some pre-existing trajectory which can be empty),
    //   route to follow (as road centerline coordinates),
    //   time horizon T,
    //   max speed, acceleration and jerk,
    //   sensor fusion structure describing the environment (e.g. other cars)
    virtual Trajectory extentTrajectory(const Car& car,
                                        const Route& route,
                                        const SensorFusion& sf,
                                        double T,
                                        double target_speed,
                                        double max_speed,
                                        double max_acceleration,
                                        double max_jerk
    ) = 0;
private:
};

/*
class StraightLinePlanner : public PathPlanner
{
public:
    virtual Trajectory getTrajectory(const Car& c);

private:
};

class CircularLinePlanner : public PathPlanner
{
public:
    virtual Trajectory getTrajectory(const Car& c);

private:
};
*/

class JMTPlanner : public PathPlanner
{
public:
    virtual Trajectory extentTrajectory(const Car& car,
                                        const Route& route,
                                        const SensorFusion& sf,
                                        double T,
                                        double target_speed,
                                        double max_speed,
                                        double max_acceleration,
                                        double max_jerk);

private:
};

#endif //PATH_PLANNING_PATHPLANNER_H

//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include "Trajectory.h"
#include "Car.h"
#include "JerkMinimizingPolynomial.h"


class PathPlanner {
public:
    virtual Trajectory getTrajectory(const Car& c) = 0;
private:
};

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

class JMTPlanner : public PathPlanner
{
public:
    virtual Trajectory getTrajectory(const Car& c);

private:
};

#endif //PATH_PLANNING_PATHPLANNER_H

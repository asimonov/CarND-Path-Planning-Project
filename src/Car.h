//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include "Trajectory.h"

class Car {
public:
    // x,y are car coordinates in global system
    Car(double x, double y, double yaw, double speed, const Trajectory& prev_traj);
    double getX() const { return _x; }
    double getY() const { return _y; }
    double getYaw() const { return _yaw; }
    double getSpeed() const { return _speed; }
    Trajectory getPrevTraj() const { return _prev_traj; }
    // translate x,y in car coordinates into global coordinates (given car position on the map)
    std::vector<double> car2global(double x, double y) const;
    // translate x,y in map coordinates into car coordinates (given car position on the map)
    std::vector<double> global2car(double x_map, double y_map) const;
private:
    Car();
    double _x;
    double _y;
//    double _s;
//    double _d;
    double _yaw;
    double _speed;
    Trajectory _prev_traj;
};


#endif //PATH_PLANNING_CAR_H

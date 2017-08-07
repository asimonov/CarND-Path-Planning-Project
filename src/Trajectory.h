//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>

// this is simplified trajectory class.
// it simply tracks x,y coordinates
// and assumes they are spaced at dt time intervals for calculation of derivatives w.r.t time (speed, acceleration, jerk etc).
// if dt is zero then derivative calculation does not work
class Trajectory {
public:
    Trajectory() {_dt=-1;}
    Trajectory(double dt);
    Trajectory(std::vector<double> x, std::vector<double> y, double dt);
    void add(double x, double y);
    std::vector<double> getX() const;
    std::vector<double> getY() const;
    double getDt() const {return _dt;}
    int getSize() const {return _x_vals.size(); }
    // recalculate (inplace) assuming constant speed of v (units/sec) and discretisation dt
    //void respace_at_constant_speed(double dt, double v);
private:
    std::vector<double> _x_vals; // x coordinate, meters
    std::vector<double> _y_vals; // y coordinate, meters
    double              _dt; // discretisation time interval, seconds
};


#endif //PATH_PLANNING_TRAJECTORY_H

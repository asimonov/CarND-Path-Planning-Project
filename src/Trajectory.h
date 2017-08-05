//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>

class Trajectory {
public:
    Trajectory();
    Trajectory(std::vector<double> x, std::vector<double> y);
    void add(double x, double y);
    std::vector<double> getX();
    std::vector<double> getY();
private:
    std::vector<double> _x_vals;
    std::vector<double> _y_vals;
};


#endif //PATH_PLANNING_TRAJECTORY_H

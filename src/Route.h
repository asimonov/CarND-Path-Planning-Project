//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_ROUTE_H
#define PATH_PLANNING_ROUTE_H

#include <string>
#include <vector>

#include "Trajectory.h"
#include "Car.h"

class Route {
public:
    Route();
    ~Route();

    void read_data(std::string map_file);

    Trajectory get_next_segments(double x, double y, double yaw, int n) const;

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    std::vector<double> get_frenet(double x, double y, double yaw) const;

    // Transform from Frenet s,d coordinates to Cartesian x,y
    std::vector<double> get_XY(double s, double d) const;

    double get_max_s() const;

private:
    // route waypoint's x,y,s and d normalized normal vectors
    std::vector<double> _waypoints_x;
    std::vector<double> _waypoints_y;
    std::vector<double> _waypoints_s;
    //std::vector<double> _waypoints_dx;
    //std::vector<double> _waypoints_dy;

    int closest_waypoint(double x, double y) const;
    int next_waypoint(double x, double y, double yaw) const;

    // internal version of get_frenet that is also used to re-generate s,d from 'splined' x,y
    std::vector<double> get_frenet2(double x, double y, int next_wp) const;

    void smooth_using_splines();
    int cyclic_index(int i) const;
};


#endif //PATH_PLANNING_ROUTE_H

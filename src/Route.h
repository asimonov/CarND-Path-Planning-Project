//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_ROUTE_H
#define PATH_PLANNING_ROUTE_H

#include <string>
#include <vector>

class Route {
public:
    Route();
    ~Route();

    void read_data(std::string map_file);

private:
    // route waypoint's x,y,s and d normalized normal vectors
    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;

};


#endif //PATH_PLANNING_ROUTE_H

//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_COORDINATE_UTILS_H
#define PATH_PLANNING_COORDINATE_UTILS_H

#include <math.h>
#include <vector>


constexpr double pi() { return M_PI; }

double deg2rad(double x);
double rad2deg(double x);

int ClosestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y);

int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, std::vector<double> maps_s, std::vector<double> maps_x, std::vector<double> maps_y);


#endif //PATH_PLANNING_COORDINATE_UTILS_H

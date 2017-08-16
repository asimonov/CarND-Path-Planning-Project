//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_COORDINATE_UTILS_H
#define PATH_PLANNING_COORDINATE_UTILS_H

#include <math.h>
#include <vector>
#include <cassert>


constexpr double pi() { return M_PI; }

double deg2rad(double x);
double rad2deg(double x);

double euclidian_distance(double x1, double y1, double x2, double y2);
double angle(double x1, double y1, double x2, double y2);

double mph2ms(double mph);
double ms2mph(double ms);

//// calculate speed from coordinates, assuming they are discretised at dt. over horizon T (from the end)
//double calc_speed(std::vector<double> coords, double dt, double T);
//
//// calculate acceleration from coordinates, assuming they are discretised at dt. over horizon T (from the end)
//double calc_acceleration(std::vector<double> coords, double dt, double T);

/*
 * A function that returns a value between 0 and 1 for x in the
        range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
 */
double logistic(double x);


#endif //PATH_PLANNING_COORDINATE_UTILS_H

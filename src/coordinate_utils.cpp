//
// Created by Alexey Simonov on 05/08/2017.
//

#include "coordinate_utils.h"

using namespace std;

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

double euclidian_distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double mph2ms(double mph) { return mph * 0.44704;}
double ms2mph(double ms) { return ms * 2.23694;}


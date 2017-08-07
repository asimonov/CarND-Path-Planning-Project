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

double angle(double x1, double y1, double x2, double y2)
{
  return atan2(y2 - y1, x2 - x1);
}


double mph2ms(double mph) { return mph * 0.44704;}
double ms2mph(double ms) { return ms * 2.23694;}

// calculate speed from coordinates, assuming they are discretised at dt. over horizon T (from the end)
double calc_speed(std::vector<double> coords, double dt, double T)
{
  int n = T/dt;
  int N = coords.size();
  assert(n>2);
  assert(N>n);
  double res = 0.0;
  for (int i=0;i<n;i++)
    res += coords[N-i-1];
  return res / (n*dt);
}

// calculate acceleration from coordinates, assuming they are discretised at dt. over horizon T (from the end)
double calc_acceleration(std::vector<double> coords, double dt, double T)
{
  int n = T/dt;
  int N = coords.size();
  assert(n>2);
  assert(N>n);
  double res = 0.0;
  for (int i=0;i<n-1;i++)
    res += (coords[N-i-1] - coords[N-i-2]) / dt;
  return res / ((n-1)*dt);
}

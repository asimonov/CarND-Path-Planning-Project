//
// Created by Alexey Simonov on 05/08/2017.
//

#include "Route.h"

#include <fstream>
#include <sstream>
#include "coordinate_utils.h"
#include <cassert>


using namespace std;

Route::Route() {}
Route::~Route() {}


double Route::get_max_s() const
{
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.57;
  return max_s;
}


void Route::read_data(string map_file_) {
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    _waypoints_x.push_back(x);
    _waypoints_y.push_back(y);
    assert(_waypoints_x.size() == _waypoints_y.size());
    _waypoints_s.push_back(s);
    assert(_waypoints_x.size() == _waypoints_s.size());
    _waypoints_dx.push_back(d_x);
    assert(_waypoints_x.size() == _waypoints_dx.size());
    _waypoints_dy.push_back(d_y);
    assert(_waypoints_x.size() == _waypoints_dy.size());
  }

  generate_splines();
}

void Route::generate_splines() {
  vector<double> s;
  vector<double> x;
  vector<double> y;
  vector<double> dx;
  vector<double> dy;

  int n = _waypoints_s.size();
  double prev_s = _waypoints_s[0] - get_max_s() + _waypoints_s[n-1];

  s.push_back(prev_s);
  x.push_back(_waypoints_x[n-1]);
  y.push_back(_waypoints_y[n-1]);
  dx.push_back(_waypoints_dx[n-1]);
  dy.push_back(_waypoints_dy[n-1]);

  s.insert(s.end(), _waypoints_s.begin(), _waypoints_s.end());
  x.insert(x.end(), _waypoints_x.begin(), _waypoints_x.end());
  y.insert(y.end(), _waypoints_y.begin(), _waypoints_y.end());
  dx.insert(dx.end(), _waypoints_dx.begin(), _waypoints_dx.end());
  dy.insert(dy.end(), _waypoints_dy.begin(), _waypoints_dy.end());

  s.push_back(get_max_s());
  x.push_back(_waypoints_x[0]);
  y.push_back(_waypoints_y[0]);
  dx.push_back(_waypoints_dx[0]);
  dy.push_back(_waypoints_dy[0]);

  _spline_x.set_points(s, x);
  _spline_y.set_points(s, y);
  _spline_dx.set_points(s, dx);
  _spline_dy.set_points(s, dy);
}

int Route::cyclic_index(int i) const
{
  int res = i;
  int cnt = 0;
  while (res<0) {
    res += _waypoints_x.size();
    assert(cnt++ < 10);
  }
  return res % _waypoints_x.size();
}


int Route::closest_waypoint(double x, double y) const
{
  double closestLen = 1e+100; // max distance
  int closestWaypoint = 0; // index of closest waypoint

  double map_x, map_y, dist;
  for (int i = 0; i < _waypoints_x.size(); i++) {
    map_x = _waypoints_x[i];
    map_y = _waypoints_y[i];
    dist = euclidian_distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}


int Route::next_waypoint(double x, double y, double yaw) const {
  int closestWaypoint = closest_waypoint(x, y);

  double map_x = _waypoints_x[closestWaypoint];
  double map_y = _waypoints_y[closestWaypoint];

  double heading = angle(x, y, map_x, map_y);

  double phi = fabs(yaw - heading);

  // what if closest waypoint is behind?
  if (phi > pi() / 4)
    closestWaypoint++;

  return cyclic_index(closestWaypoint);
}


// internal version of get_frenet
std::vector<double> Route::get_frenet2(double x, double y, int next_wp) const
{
  int prev_wp;
  prev_wp = cyclic_index(next_wp - 1);

  double n_x = _waypoints_x[next_wp] - _waypoints_x[prev_wp];
  double n_y = _waypoints_y[next_wp] - _waypoints_y[prev_wp];
  double x_x = x - _waypoints_x[prev_wp];
  double x_y = y - _waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;
  double sign_len = (x_x * n_x + x_y * n_y) / sqrt(n_x * n_x + n_y * n_y);

  double frenet_s = _waypoints_s[prev_wp];
  double s_dist = _waypoints_s[next_wp] - _waypoints_s[prev_wp];
  if (s_dist<0)
    s_dist += get_max_s();
  double xy_dist = euclidian_distance(_waypoints_x[prev_wp], _waypoints_y[prev_wp], _waypoints_x[next_wp], _waypoints_y[next_wp]);
  frenet_s += (s_dist / xy_dist) * sign_len;
  if (frenet_s < (_waypoints_s[_waypoints_s.size()-1] - get_max_s()) )
    frenet_s += get_max_s();

  double x_adj = _spline_x(frenet_s);
  double y_adj = _spline_y(frenet_s);

  double frenet_d = euclidian_distance(x, y, x_adj, y_adj);

  if (frenet_s < 0 )
    frenet_s += get_max_s();

  return {frenet_s, frenet_d};
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> Route::get_frenet(double x, double y, double yaw) const
{
  int next_wp = next_waypoint(x, y, yaw);
  return get_frenet2(x, y, next_wp);
}


// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> Route::get_XY(double s, double d) const
{
  int prev_wp = -1;

  if (s>_waypoints_s[_waypoints_s.size()-1])
    s -= get_max_s();

  double x0 = _spline_x(s);
  double y0 = _spline_y(s);
  double dx = _spline_dx(s);
  double dy = _spline_dy(s);
  double x = x0 + d * dx;
  double y = y0 + d * dy;

  return {x, y};
}


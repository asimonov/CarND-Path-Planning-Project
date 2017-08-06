//
// Created by Alexey Simonov on 05/08/2017.
//

#include "Route.h"

#include <fstream>
#include <sstream>
#include "coordinate_utils.h"
#include <cassert>
#include "spline.h"

using namespace std;

Route::Route() {}
Route::~Route() {}

void Route::read_data(string map_file_) {

  // The max s value before wrapping around the track back to 0
  //double max_s = 6945.554;

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
    //_waypoints_dx.push_back(d_x);
    //assert(_waypoints_x.size() == _waypoints_dx.size());
    //_waypoints_dy.push_back(d_y);
    //assert(_waypoints_x.size() == _waypoints_dy.size());
  }

  smooth_using_splines();
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

void Route::smooth_using_splines()
{
  vector<double> new_waypoints_x;
  vector<double> new_waypoints_y;

  // do not try to smooth if we have just few segments
  if (_waypoints_x.size() < 3)
    return;

  // here we will iterate over existing waypoints
  // for each waypoint we will take next NUM_SEGMENTS segments of the route
  // cast their coordinates into car system (so x is out and y is to the left of the car)
  // then spline them in the car coordinates
  // then cast them back to global coordinates
  // and create extra NUM_INTRA_POINTS between current and next waypoints from spline
  const int NUM_SEGMENTS = 3; // number of segments to smooth
  const int NUM_INTRA_POINTS = 5; // number of intermediary waypoints we create in each segment
  for (int current_wp=0; current_wp<_waypoints_x.size(); current_wp++)
  {
    vector<double> spline_x;
    vector<double> spline_y;

    double curr_x_gl = _waypoints_x[current_wp];
    double curr_y_gl = _waypoints_y[current_wp];
    int next_wp = cyclic_index(current_wp + 1);
    double next_x_gl = _waypoints_x[next_wp];
    double next_y_gl = _waypoints_y[next_wp];

    double angle = atan2(next_y_gl - curr_y_gl, next_x_gl - curr_x_gl);

    // create car at the current waypoint pointing to the next
    Car c(curr_x_gl, curr_y_gl, angle, 0, Trajectory());

    // define spline inputs
    for (int i=0;i<NUM_SEGMENTS;i++)
    {
      int idx = cyclic_index(current_wp - 1 /*actually, take one segment prior in our spline*/ + i);
      auto xy = c.global2car(_waypoints_x[idx], _waypoints_y[idx]);
      spline_x.push_back(xy[0]);
      spline_y.push_back(xy[1]);
    }

    // interpolate current segment using spline
    tk::spline s;
    s.set_points(spline_x, spline_y, true/*cubic*/);
    auto xy1 = c.global2car(_waypoints_x[current_wp], _waypoints_y[current_wp]);
    auto xy2 = c.global2car(_waypoints_x[next_wp], _waypoints_y[next_wp]);
    double dx = (xy2[0] - xy1[0]) / (NUM_INTRA_POINTS);
    assert(dx > 0.0); // in car coordinates the route should not turn back
    for (int i=0; i<NUM_INTRA_POINTS; i++) {
      double x = xy1[0] + i * dx;
      double y = s(x);
      auto xy = c.car2global(x,y);
      new_waypoints_x.push_back(xy[0]);
      new_waypoints_y.push_back(xy[1]);
    }
  }

  // replace waypoints with smoothed waypoints
  _waypoints_x = new_waypoints_x;
  _waypoints_y = new_waypoints_y;
  assert(_waypoints_x.size() == _waypoints_y.size());

  //recalculate frenet s,d for new waypoints
  vector<double> new_s;
  //vector<double> new_d;
  for (int i=0; i<_waypoints_x.size(); i++)
  {
    auto f = get_frenet2(_waypoints_x[i], _waypoints_y[i], i+1);
    new_s.push_back(f[0]);
    //new_d.push_back(f[1]);
  }
  _waypoints_s = new_s;
  //_waypoints_d = new_d;
  assert(_waypoints_s.size() == _waypoints_x.size());

  /* debug code for strange behaviour
   *
  auto fr = get_frenet(773, 1135, 352);
  auto xy = get_XY(fr[0],fr[1]);
  auto fr2 = get_frenet(778, 1135, 352);
  auto xy2 = get_XY(fr2[0],fr2[1]);
  auto fr3 = get_frenet(780, 1135, 352);
  auto xy3 = get_XY(fr3[0],fr3[1]);
  auto fr4 = get_frenet(785, 1135, 352);
  auto xy4 = get_XY(fr4[0],fr4[1]);
  auto fr4_ = get_frenet(784.6, 1135.57, 352);
  auto xy4_ = get_XY(fr4_[0],fr4_[1]);
  auto xy4_2 = get_XY(fr4_[0],fr4_[1]+2.0);
  Trajectory tr = get_next_segments(Car(741.16,1135.88,352,0,Trajectory()), 15);
  Trajectory tr_d;
  for (int i=0; i<tr.getX().size(); i++) {
    auto fr = get_frenet(tr.getX()[i], tr.getY()[i], 352);
    fr[1] += 2.0; // be in middle of left lane
    auto xy = get_XY(fr[0], fr[1]);
    tr_d.add(xy[0], xy[1]);
  }
  //tr_d.respace_at_constant_speed(dt_s, mph2ms(45.0));

  int d = 0;
   */
}



// find next n waypoints to follow
// given car position
Trajectory Route::get_next_segments(double x, double y, double yaw, int n)
{
  Trajectory tr;

  long next_wp = next_waypoint(x, y, yaw);
  //next_wp = cyclic_index(next_wp-1);

  for (int i=0; i<n; i++)
  {
    tr.add(_waypoints_x[next_wp], _waypoints_y[next_wp]);
    next_wp = cyclic_index(next_wp + 1);
  }

  return tr;
}


int Route::closest_waypoint(double x, double y)
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


int Route::next_waypoint(double x, double y, double yaw) {
  int closestWaypoint = closest_waypoint(x, y);

  double map_x = _waypoints_x[closestWaypoint];
  double map_y = _waypoints_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(yaw - heading);

  if (angle > pi() / 4)
    closestWaypoint++;

  return cyclic_index(closestWaypoint);
}


// internal version of get_frenet that is also used to re-generate s,d from 'splined' x,y
std::vector<double> Route::get_frenet2(double x, double y, int next_wp)
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

  double frenet_d = euclidian_distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - _waypoints_x[prev_wp];
  double center_y = 2000 - _waypoints_y[prev_wp];
  double centerToPos = euclidian_distance(center_x, center_y, x_x, x_y);
  double centerToRef = euclidian_distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += euclidian_distance(_waypoints_x[i], _waypoints_y[i], _waypoints_x[i + 1], _waypoints_y[i + 1]);
  }

  frenet_s += euclidian_distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> Route::get_frenet(double x, double y, double yaw)
{
  int next_wp = next_waypoint(x, y, yaw);
  return get_frenet2(x, y, next_wp);
}


// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> Route::get_XY(double s, double d)
{
  int prev_wp = -1;

  while (s >= _waypoints_s[prev_wp + 1] && (prev_wp < (int) (_waypoints_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = cyclic_index(prev_wp + 1);
  prev_wp = cyclic_index(prev_wp); // to fix bug when s=0 for example

  double heading = atan2((_waypoints_y[wp2] - _waypoints_y[prev_wp]), (_waypoints_x[wp2] - _waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - _waypoints_s[prev_wp]);

  double seg_x = _waypoints_x[prev_wp] + seg_s * cos(heading);
  double seg_y = _waypoints_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}


//
// Created by Alexey Simonov on 05/08/2017.
//

#include "PathPlanner.h"
#include "coordinate_utils.h"
#include "JerkMinimizingPolynomial.h"

/*
// straight line and constant velosity trajectory
Trajectory StraightLinePlanner::getTrajectory(const Car& c)
{
  Trajectory tr;
  double dist_inc = 0.5;
  double x, y;
  for (int i = 0; i < 50; i++) {
    x = c.getX() + (dist_inc * i) * cos(deg2rad(c.getYaw()));
    y = c.getY() + (dist_inc * i) * sin(deg2rad(c.getYaw()));
    tr.add(x,y);
  }
  return tr;
}


// constant angle and velosity trajectory
Trajectory CircularLinePlanner::getTrajectory(const Car& c)
{
  Trajectory tr;

  double dist_inc = 0.5;
  double x, y;

  Trajectory pr_tr = c.getPrevTraj();

  double pos_x;
  double pos_y;
  double angle;
  int path_size = pr_tr.getX().size();

  if(path_size == 0)
  {
    pos_x = c.getX();
    pos_y = c.getY();
    angle = deg2rad(c.getYaw());
  }
  else
  {
    for(int i = 0; i < path_size; i++)
      tr.add(pr_tr.getX()[i], pr_tr.getY()[i]);

    pos_x = pr_tr.getX()[path_size-1];
    pos_y = pr_tr.getY()[path_size-1];

    double pos_x2 = pr_tr.getX()[path_size-2];
    double pos_y2 = pr_tr.getY()[path_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }

  for(int i = 0; i < 50-path_size; i++)
  {
    x = pos_x+(dist_inc)*cos(angle+(i+1)*(pi() / 100));
    y = pos_y+(dist_inc)*sin(angle+(i+1)*(pi() / 100));
    tr.add(x,y);
    pos_x += (dist_inc)*cos(angle+(i+1)*(pi() / 100));
    pos_y += (dist_inc)*sin(angle+(i+1)*(pi() / 100));
  }

  return tr;
}
*/


Trajectory JMTPlanner::extentTrajectory(const Car& car,
                                    const Route& route,
                                    const SensorFusion& sf,
                                    double T,
                                    double max_speed,
                                    double max_acceleration,
                                    double max_jerk)
{
  Trajectory trajectory = car.getPrevTraj(); // final trajectory in (x,y), spaced at time discretisation frequency

  // if planned trajectory is longer than time horizon, nothing to do
  if (trajectory.getTotalT() > T)
    return trajectory;

  int len = trajectory.getSize();

  // get final state of existing trajectory
  double curr_x = car.getX();
  double curr_y = car.getY();
  double curr_yaw = car.getYaw();
  double curr_speed = car.getSpeed();
  double curr_acceleration = 0.0;
  if (len)
  {
    auto xy = trajectory.getFinalXY();
    curr_x = xy[0];
    curr_y = xy[1];
    if (len>1) {
      curr_speed = trajectory.getFinalSpeed();
      curr_yaw = trajectory.getFinalYaw();
    }
    if (len>2)
      curr_acceleration = trajectory.getFinalAcceleration();
  }
  auto fr = route.get_frenet(curr_x, curr_y, car.getYaw());
  double curr_s = fr[0];
  double curr_d = fr[1];

  // repeat until we reach time horizon
  while (trajectory.getTotalT() < T)
  {
    // get the next waypoint on the road centerline
    Trajectory waypoints = route.get_next_segments(curr_x, curr_y, curr_yaw, 1);
    assert(waypoints.getX().size() > 0);
    double next_x_center = waypoints.getX()[0];
    double next_y_center = waypoints.getY()[0];
    // calculate next waypoint s,d for position in left lane
    auto fr_center = route.get_frenet(next_x_center, next_y_center, curr_yaw);
    double lane_width = 4.0;
    auto fr_left_lane = fr_center;
    fr_left_lane[1] += lane_width / 2.0; // be in the middle of left lane
    double next_s = fr_left_lane[0];
    double next_d = fr_left_lane[1];

    // we find trajectory in (s,d)
    if (next_s < curr_s)
      next_s += route.get_max_s();
    double dist_s = next_s - curr_s;
    assert(dist_s>0);

    double plan_time = T - trajectory.getTotalT();
    double use_acceleration = max_acceleration / 2.0;
    double possible_speed = curr_speed + plan_time * use_acceleration;
    double next_speed = possible_speed;
    double next_acceleration = use_acceleration;
    if (next_speed > max_speed) {
      next_speed = max_speed;
      next_acceleration = ( next_speed - curr_speed ) / plan_time;
    }

    // use JMT to find good trajectory over plan_time
    JerkMinimizingPolynomial jmt_s({curr_s, curr_speed, curr_acceleration},
                                   {next_s, next_speed, next_acceleration}, plan_time);
    // discretise the JMT suggested path
    int n_steps = floor(plan_time / trajectory.getDt());
    for (int j=0;j<n_steps; j++)
    {
      double s = jmt_s.eval(j*trajectory.getDt());
      auto xy = route.get_XY(s, next_d);
      trajectory.add(xy[0], xy[1]);
    }

    // update currXXX variables
    auto xy = trajectory.getFinalXY();
    curr_x = xy[0];
    curr_y = xy[1];
    curr_yaw = trajectory.getFinalYaw();
    curr_speed = trajectory.getFinalSpeed();
    curr_acceleration = trajectory.getFinalAcceleration();
    auto fr = route.get_frenet(curr_x, curr_y, curr_yaw);
    curr_s = fr[0];
    curr_d = fr[1];
  }

  return trajectory;
}



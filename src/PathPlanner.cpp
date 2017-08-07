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
  int len = trajectory.getSize();
  double planned_t = trajectory.getDt() * len;

  // if planned trajectory is longer than time horizon, nothing to do
  if (planned_t > T)
    return trajectory;

  double curr_x = car.getX();
  double curr_y = car.getY();
  double curr_yaw = car.getYaw();
  auto fr = route.get_frenet(curr_x, curr_y, car.getYaw());
  double curr_s = fr[0];
  double curr_d = fr[1];

  // repeat until we reach time horizon
  while (planned_t < T)
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


    // TODO increase planned_t, update currXXX variables
  }

/*
 *
          double s_prev = fr[0];
          double s_prev_dot = 0;
          double s_prev_ddot = 0;
          double d_prev = fr[1];
          double d_prev_dot = 0;
          double d_prev_ddot = 0;
          double t_total = 0;

          Trajectory waypoints = route.get_next_segments(c.getX(), c.getY(), c.getYaw(), 15);
          int n_wp = waypoints.getX().size();
          for (int i=0; i<n_wp; i++) {
            fr = route.get_frenet(waypoints.getX()[i], waypoints.getY()[i], c.getYaw());
            fr[1] += 2.0; // be in middle of left lane
            double s_new = fr[0];
            double d_new = fr[1];

            double s_dist = s_new - s_prev;
            assert(s_dist>0);
            double delta_v = target_speed_ms - s_prev_dot;
            double used_acceleration = max_acceleration/2.0;
            double T = abs(delta_v) / used_acceleration; // time horizon over which we can get to desired speed
//            if (T>time_horizon_s) {
//              T = time_horizon_s;
//            }
            double s_new_dot = s_prev_dot + T*used_acceleration;
            double s_new_ddot = 0.0;
            JerkMinimizingPolynomial jmt_s({s_prev, s_prev_dot, s_prev_ddot}, {s_new, s_new_dot, s_new_ddot}, T);

            int n_steps = T/dt_s;
            for (int j=0;j<n_steps; j++)
            {
              double s = jmt_s.eval(j*dt_s);
              tr_left_lane_frenet.add(s, d_new);
            }

            s_prev = s_new;
            s_prev_dot = s_new_dot;
            s_prev_ddot = s_new_ddot;
            d_prev = d_new;
            d_prev_dot = 0;
            d_prev_ddot = 0;
            t_total += T;
            if (t_total > time_horizon_s)
              break;
          }

          n_wp = tr_left_lane_frenet.getX().size();
          for (int i=0; i<n_wp; i++) {
            auto xy = route.get_XY(tr_left_lane_frenet.getX()[i], tr_left_lane_frenet.getY()[i]);
            tr.add(xy[0],xy[1]);
          }

 */


  return trajectory;
}



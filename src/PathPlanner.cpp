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

int sgn(double val) {
  return (0.0 < val) - (val < 0.0);
}

Trajectory JMTPlanner::extentTrajectory(const Car& car,
                                    const Route& route,
                                    const SensorFusion& sf,
                                    double T,
                                    double target_speed,
                                    double max_speed,
                                    double max_acceleration,
                                    double max_jerk)
{
  Trajectory trajectory = car.getPrevTraj(); // final trajectory in (x,y), spaced at time discretisation frequency

  // if planned trajectory is longer than time horizon, nothing to do
  if (trajectory.getTotalT() > T)
    return trajectory;

  // get final state of existing trajectory
  double curr_x = car.getX();
  double curr_y = car.getY();
  double curr_yaw = car.getYaw();
  double curr_speed = car.getSpeed();
  double curr_acceleration = 0.0;
  double curr_jerk = 0.0;
  int len = trajectory.getSize();
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
    if (len>3)
      curr_jerk = trajectory.getFinalJerk();
  }
  auto fr = route.get_frenet(curr_x, curr_y, car.getYaw());
  double curr_s = fr[0];
  double curr_d = fr[1];

  // get planning horizon possible conditions
  double plan_time = T - trajectory.getTotalT();
  double delta_speed = target_speed - curr_speed; // can be negative

  double safety_factor = 0.5;

  double usable_jerk = max_jerk * safety_factor;

  double usable_acceleration = max_acceleration * safety_factor;
  double possible_acceleration = usable_jerk / plan_time;
  if (possible_acceleration < usable_acceleration)
    usable_acceleration = possible_acceleration;

  double possible_speed = curr_speed
                          + sgn(delta_speed) * usable_acceleration * plan_time
                          + sgn(delta_speed) * 0.5 * usable_jerk   * plan_time * plan_time;
  if (possible_speed > max_speed)
    possible_speed = max_speed;
  if (possible_speed < 0.0)
    possible_speed = target_speed;
  if (possible_speed >= target_speed && delta_speed>=0)
    possible_speed = target_speed;
  if (possible_speed < target_speed && delta_speed<0)
    possible_speed = target_speed;

  // possible distance can be longer than we need, but this is fine
  double possible_distance = curr_speed * plan_time
                             + 0.5 * usable_acceleration * plan_time * plan_time
                             + (1./6.) * usable_jerk * plan_time * plan_time * plan_time;
  assert(possible_distance>0);

  // use JMT to find good trajectory over plan_time
  JerkMinimizingPolynomial jmt_s({0,                 curr_speed,     curr_acceleration},
                                 {possible_distance, possible_speed, 0.0}, plan_time);
  // discretise the JMT suggested path
  int n_steps = floor(plan_time / trajectory.getDt());
  std::vector<double> svec;
  for (int j=0;j<n_steps; j++)
  {
    double s = jmt_s.eval(j*trajectory.getDt());
    auto xy = route.get_XY(curr_s + s, curr_d);
    trajectory.add(xy[0], xy[1]);
    svec.push_back(s);
  }

  return trajectory;
}



//
// Created by Alexey Simonov on 05/08/2017.
//

#include "PathPlanner.h"
#include "coordinate_utils.h"
#include "JerkMinimizingPolynomial.h"
#include <iostream>

using namespace std;


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

//int sgn(double val) {
//  return (0.0 < val) - (val < 0.0);
//}


// extend trajectory given by planning_time
// car state is assumed to be as of end of given trajectory
// car speed, acceleration and predictions are set by behaviour planning
Trajectory JMTPlanner::extendTrajectory(const Car& car,
                                        const Trajectory& trajectory,
                                    const Route& route,
                                    double planning_time,
                                    double lane_width,
                                    double target_speed,
                                    double max_speed,
                                    double max_acceleration,
                                    double max_jerk)
{

  // get final state of existing trajectory
  double curr_x = car.getX();
  double curr_y = car.getY();
  double curr_yaw = car.getYaw();
  double curr_speed = car.getSpeed();
  double curr_acceleration = car.getAcceleration();
  double curr_jerk = 0.0;
  auto fr = trajectory.getFinalFrenet();
  double curr_s = fr[0];
  double curr_d = fr[1];

  // get predicted final state from behaviour planner
  Car egoPlanned = car.advance(planning_time);
  int planned_lane = egoPlanned.get_target_lane();
  double next_d = 2.0 + planned_lane * lane_width;
  if (planned_lane>1)
    next_d -= 0.3; // move rightmost lane center by 30 cm to avoid simulator bugs
  double next_s = egoPlanned.getS();
  double next_speed = egoPlanned.getSpeed();
  double next_acc = egoPlanned.getAcceleration();

  // define grid of possible T, s, d values to then generate JMT trajectories and choose those with lowest cost
  vector<double> T_values = {planning_time};
//  if (planning_time-1>1)
//    T_values.push_back(planning_time-1);
//  T_values.push_back(planning_time+1);
//  // add more time horizons to 8 seconds extra, in steps of 0.5 secs
//  for (int i=0; i<5; i++)
//    T_values.push_back( T + (i+1)*1.0 );
  vector<double> s_values = {next_s};
  // add more s horizons (from current s) to +=10 meters, in steps of 1 meters
  const double step = 1.0;
  for (int i=0; i<10; i++)
  {
//    if (next_s-(i+1)*step > 0)
//      s_values.push_back( next_s -(i+1)*step );
//    if (next_s+(i+1)*step > 0)
//      s_values.push_back( next_s +(i+1)*step );
  }
//  vector<double> d_values = {2.0, 6.0, 9.5}; // centers of left, center and right lanes
  vector<double> d_values = {next_d};

//  T_values = {T};
//  s_values = {100};
//  d_values = {curr_d};

  // generate trajectories and find one with minimal cost
  Trajectory best_trajectory = trajectory;
  double best_cost = 1e+10; // unrealistically bad cost
  for (double sample_t : T_values)
  {
    for (double sample_s : s_values)
    {
      for (double sample_d : d_values)
      {
        // use JMT to find good trajectory in s
//        double v_s = curr_speed;// * cos(curr_yaw);
//        double a_s = 0.0;//curr_acceleration;// * cos(curr_yaw);
//        double target_v_s = target_speed;
//        double target_a_s = 0.0;
        JerkMinimizingPolynomial jmt_s({curr_s,   curr_speed, curr_acceleration},
                                       {sample_s, next_speed, next_acc},
                                       sample_t);

        // use JMT to find good trajectory in d
//        double v_d = 0.0;//curr_speed * sin(curr_yaw);
//        double a_d = 0.0;//curr_acceleration * sin(curr_yaw);
//        double target_v_d = 0.0;
//        double target_a_d = 0.0;
        JerkMinimizingPolynomial jmt_d({curr_d,   0.0, 0.0},
                                       {sample_d, 0.0, 0.0},
                                       sample_t);

        // discretise the JMT suggested path
        Trajectory sample_tr = trajectory; // copy existing trajectory
        double dt = trajectory.getDt();
        int n_steps = floor(sample_t / dt);
        bool first = true;
        double s;
        double d;
        for (int j=1;j<n_steps; j++)
        {
          s = jmt_s.eval(double(j)*dt);
          d = jmt_d.eval(j*dt);
          auto xy = route.get_XY(s, d);
          sample_tr.add(xy[0], xy[1]);
//          if (first) {
//            cout<<"old s: "<<curr_s<<" new s: "<<s<<" old d: "<<curr_d<<" new d: "<<d<<endl;
//            cout<<"old x: "<<curr_x<<" new x: "<<xy[0]<<" old y: "<<curr_y<<" new y: "<<xy[1]<<endl;
//            first = false;
//          }
        }
        sample_tr.storeFinalFrenet(s,d);
        // estimate trajectory cost and update if it's best we've seen so far
        double cost = sample_tr.getCost(sample_t, sample_s, target_speed,
                                        max_speed, max_acceleration, max_jerk,
                                        route);
        if (cost < best_cost)
        {
          best_trajectory = sample_tr;
          best_cost = cost;
        }
      }
    }
  }

  return best_trajectory;
}



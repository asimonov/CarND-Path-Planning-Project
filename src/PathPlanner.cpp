//
// Created by Alexey Simonov on 05/08/2017.
//

#include "PathPlanner.h"
#include "coordinate_utils.h"
#include "JerkMinimizingPolynomial.h"
#include <iostream>

using namespace std;

// Trajectory planner

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
//  // add more time horizons, in steps of 0.5 secs
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
  vector<double> d_values = {next_d};

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
        JerkMinimizingPolynomial jmt_s({curr_s,   curr_speed, curr_acceleration},
                                       {sample_s, next_speed, next_acc},
                                       sample_t);

        // use JMT to find good trajectory in d
        JerkMinimizingPolynomial jmt_d({curr_d,   0.0, 0.0},
                                       {sample_d, 0.0, 0.0},
                                       sample_t);

        // discretise the JMT suggested path
        Trajectory sample_tr = trajectory; // copy existing trajectory
        double dt = trajectory.getDt();
        int n_steps = floor(sample_t / dt);
        double s;
        double d;
        for (int j=1;j<n_steps; j++)
        {
          s = jmt_s.eval(double(j)*dt);
          d = jmt_d.eval(j*dt);
          auto xy = route.get_XY(s, d);
          sample_tr.add(xy[0], xy[1]);
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



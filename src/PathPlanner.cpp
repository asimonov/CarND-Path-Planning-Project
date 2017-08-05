//
// Created by Alexey Simonov on 05/08/2017.
//

#include "PathPlanner.h"
#include "coordinate_utils.h"

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

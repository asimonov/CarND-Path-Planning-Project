//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_TRAJECTORY_H
#define PATH_PLANNING_TRAJECTORY_H

#include <vector>
#include <sstream>
#include <string>

// this is simplified trajectory class.
// it simply tracks x,y coordinates
// and assumes they are spaced at dt time intervals for calculation of derivatives w.r.t time (speed, acceleration, jerk etc).
// if dt is zero then derivative calculation does not work
class Trajectory {
public:
    Trajectory();
    Trajectory(double dt);
    Trajectory(std::vector<double> x, std::vector<double> y, double dt);
    //Trajectory(const Trajectory& other) = delete;

    void add(double x, double y);

    std::vector<double> getX() const;
    std::vector<double> getY() const;

    double getDt() const {return _dt;}
    int getSize() const {return _x_vals.size(); }
    double getTotalT() const;
    double getTotalDistance() const { return _total_distance; }

    std::vector<double> getStartXY() const;
    std::vector<double> getFinalXY() const;

    double getStartYaw() const;
    double getFinalYaw() const;

    double getStartSpeed() const;
    double getFinalSpeed() const;
    double getMinSpeed() const;
    double getMaxSpeed() const;

    double getStartAcceleration() const;
    double getFinalAcceleration() const;
    double getMinAcceleration() const;
    double getMaxAcceleration() const;

    double getStartJerk() const;
    double getFinalJerk() const;
    double getMinJerk() const;
    double getMaxJerk() const;
    double getTotalSquaredJerk() const;

    double getCost(double target_time, double target_distance, double target_speed,
                   double max_speed, double max_acceleration, double max_jerk) const;
    void dump_to_file(const std::string& filename) const;

private:
    std::vector<double> _x_vals; // x coordinate, meters
    std::vector<double> _y_vals; // y coordinate, meters
    std::vector<double> _dist; // euclidian distances between adjacent (x,y)
    double              _total_distance;
    std::vector<double> _speed; // speed between adjacent (x,y)
    double              _min_speed;
    double              _max_speed;
    std::vector<double> _acceleration; // acceleration between adjacent speed points
    double              _min_acceleration;
    double              _max_acceleration;
    std::vector<double> _jerk; // jerk between adjacent acceleration points
    double              _min_jerk;
    double              _max_jerk;
    std::vector<double> _heading; // heading between adjacent (x,y), from beginning of the segment
    double              _dt; // discretisation time interval, seconds
    mutable std::string         _cost_dump_str;
};


#endif //PATH_PLANNING_TRAJECTORY_H

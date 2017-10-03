//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>
#include <string>
#include <ostream>

enum Maneuvre
{
    CONSTANT_SPEED,
    KEEP_LANE,
    PREPARE_CHANGE_LANE,
    CHANGE_LANE
};

// car location predictions (lane, s_coordinate) spaced at dt
typedef std::vector<std::pair<int, double>> predictions_type;


class Car {
public:
    static int getEgoID() {return -12345;}

    // x,y are car coordinates in global system. s,d are equivalent frenet coordinates
    Car(int id,
        double x, double y, double yaw,
        double s, double d,
        int lane,
        double speed, double acceleration,
        double target_speed,
        double max_speed, double max_acceleration
    );

    // return a copy with evolved state of this car (s,speed) to time T
    Car advance(double T) const;

    int getID() const { return _id; }
    double getX() const { return _x; }
    double getY() const { return _y; }
    double getYaw() const { return _yaw; }
    double getS() const { return _s; }
    double getD() const { return _d; }
    int getLane() const { return _lane; }
    double getSpeed() const { return _speed; }
    double getMaxSpeed() const { return _max_speed; }
    double getAcceleration() const { return _acceleration; }
    double getMaxAcceleration() const { return _max_acceleration; }
    double getTargetSpeed() const { return _target_speed; }
    std::pair<Maneuvre, int> getState() const { return _state; }
    double getLength() const {return LENGTH;}
    double getWidth() const {return WIDTH;}

    // sets the state (maneuvre and goal lane) and re-calculates parameters to achieve it
    void setState(std::pair<Maneuvre, int>& state, const std::vector<Car>& other_cars, double maneuvre_time);
    // updates internal state with trajectory spaced at dt up till T
    void generate_predictions(double T, double dt);
    const predictions_type& get_predictions() const;
    // get planned time horizon
    double get_target_time() const;
    int    get_target_lane() const;

    void dumpToStream(const std::string& filename) const;

    // translate x,y in car coordinates into global coordinates (given car position on the map)
    std::vector<double> car2global(double x, double y) const;
    // translate x,y in map coordinates into car coordinates (given car position on the map)
    std::vector<double> global2car(double x_map, double y_map) const;
private:
    Car();
    double LENGTH=5.0;
    double WIDTH=2.5;
    int _id;
    double _x;
    double _y;
    double _yaw;
    double _s;
    double _d;
    int _lane;
    double _speed;
    double _acceleration;
    // behavioural targets
    double _target_speed;
    double _max_speed;
    double _max_acceleration;

    std::pair<Maneuvre, int> _state; // string representing state (KL, LC, PLC) and target_lane
    // internal predicted state
    double _predictions_dt;
    predictions_type _predictions; // predicted state (lane,s) spaced at _predictions_dt
    double maxAccelerationForLane(const std::vector<Car>& other_cars, double maneuvre_time);
};


#endif //PATH_PLANNING_CAR_H

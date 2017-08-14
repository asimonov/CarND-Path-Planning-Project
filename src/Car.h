//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>
#include <string>

class Car {
public:
    static int getEgoID() {return -12345;}

    // x,y are car coordinates in global system. s,d are equivalent frenet coordinates
    Car(int id,
        double x, double y, double yaw,
        double s, double d,
        int lane,
        double speed, double acceleration,
        double target_speed
    );

    // evolve state of this car (s,speed) to time T
    Car advance(double T);

    int getID() const { return _id; }
    double getX() const { return _x; }
    double getY() const { return _y; }
    double getYaw() const { return _yaw; }
    double getS() const { return _s; }
    double getD() const { return _d; }
    int getLane() const { return _lane; }
    double getSpeed() const { return _speed; }
    double getAcceleration() const { return _acceleration; }
    double getTargetSpeed() const { return _target_speed; }
    std::string getState() const { return _state; }
    int getTargetLane() const {return _target_lane;}
    double getLength() const {return LENGTH;}
    double getWidth() const {return WIDTH;}


    void generate_predictions(double T, double dt);

    void setState(std::string& state) { _state = state; }
    void setTargetLane(int target_lane) {_target_lane=target_lane; }
    void setAcceleration(double a) {_acceleration=a;}

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
    std::string _state;
    int _target_lane;
    // internal predicted state
    std::vector<std::pair<int, double>> _predictions; // predicted state (lane,s) spaced at _predictions_dt
    double _predictions_dt;
};


#endif //PATH_PLANNING_CAR_H

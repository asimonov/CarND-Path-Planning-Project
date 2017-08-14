//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_CAR_H
#define PATH_PLANNING_CAR_H

#include <vector>

class Car {
public:
    static int getEgoID() {return -12345;}

    // x,y are car coordinates in global system. s,d are equivalent frenet coordinates
    Car(int id,
        double x, double y, double yaw,
        double s, double d,
        int lane,
        double speed, double acceleration);

    // evolve state of this car (s,speed) to time T
    Car advance(double T);

    double getX() const { return _x; }
    double getY() const { return _y; }
    double getYaw() const { return _yaw; }
    double getSpeed() const { return _speed; }

    // translate x,y in car coordinates into global coordinates (given car position on the map)
    std::vector<double> car2global(double x, double y) const;
    // translate x,y in map coordinates into car coordinates (given car position on the map)
    std::vector<double> global2car(double x_map, double y_map) const;
private:
    Car();
    int _id;
    double _x;
    double _y;
    double _yaw;
    double _s;
    double _d;
    int _lane;
    double _speed;
    double _acceleration;
};


#endif //PATH_PLANNING_CAR_H

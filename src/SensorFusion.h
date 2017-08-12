//
// Created by Alexey Simonov on 07/08/2017.
//

#ifndef PATH_PLANNING_SENSORFUSION_H
#define PATH_PLANNING_SENSORFUSION_H

#include <vector>
#include "Car.h"

class SensorFusion {
public:
    void add(const Car& car);
    int size() const;
    const Car& getCar(int i) const;
private:
    std::vector<Car> _cars;
};


#endif //PATH_PLANNING_SENSORFUSION_H

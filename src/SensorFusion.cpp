//
// Created by Alexey Simonov on 07/08/2017.
//

#include "SensorFusion.h"
#include <cassert>

void SensorFusion::add(const Car& car)
{
  _cars.push_back(car);
}

int SensorFusion::size() const
{
  return _cars.size();
}

const Car& SensorFusion::getCar(int i) const
{
  assert(i>=0 && i<_cars.size());
  return _cars[i];
}


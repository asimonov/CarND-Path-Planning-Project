//
// Created by Alexey Simonov on 05/08/2017.
//

#ifndef PATH_PLANNING_LOG_UTILS_H
#define PATH_PLANNING_LOG_UTILS_H

#include <string>
#include "Trajectory.h"

//return time since first call (remembered in static variable) in milliseconds
unsigned long ts_ms();

//return time since first call in ms, formatted as string
std::string ts_ms_str();



#endif //PATH_PLANNING_LOG_UTILS_H

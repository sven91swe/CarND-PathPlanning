//
// Created by sven on 2017-11-29.
//

#ifndef PATH_PLANNING_VEHICLECONTROL_H
#define PATH_PLANNING_VEHICLECONTROL_H

#include <vector>
#include "VehicleBehavior.h"
#include "support.h"

using namespace std;

class VehicleControl {
public:
    vector<double> getNextX();
    vector<double> getNextY();
    void calculateNextPath(vehicleAndMapState &state);
    void init();

private:
    VehicleBehavior* currentBehavior;
};


#endif //PATH_PLANNING_VEHICLECONTROL_H

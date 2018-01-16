//
// Created by sven on 2017-11-30.
//

#ifndef PATH_PLANNING_CHANGELANE_H
#define PATH_PLANNING_CHANGELANE_H

#include "VehicleBehavior.h"
#include "support.h"


class ChangeLane: public VehicleBehavior {
public:
    ChangeLane(int direction);
    VehicleBehavior* nextBehaviorAndCalculateNextPath(vehicleAndMapState &state, bool firstCall = true);
    bool possibleToChangeToBehavior(vehicleAndMapState &state);

private:
    vector<vector<double>> possiblePath(vehicleAndMapState &state, double finalV_mph);

    int direction = 0;
};


#endif //PATH_PLANNING_CHANGELANE_H

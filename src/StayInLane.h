//
// Created by sven on 2017-11-30.
//

#ifndef PATH_PLANNING_STAYINLANE_H
#define PATH_PLANNING_STAYINLANE_H

#include "VehicleBehavior.h"

class StayInLane: public VehicleBehavior {
public:
    VehicleBehavior* nextBehaviorAndCalculateNextPath(vehicleAndMapState &state, bool firstCall = true);

private:
    bool startUp = true;
    vector<vector<double>> possiblePath(vehicleAndMapState &state, double finalV_mph);

    vector<vector<double>> possiblePath2(vehicleAndMapState &state, double finalV_mph);

    vector<vector<double>> possiblePath3(vehicleAndMapState &state, double finalV_mph);
};


#endif //PATH_PLANNING_STAYINLANE_H

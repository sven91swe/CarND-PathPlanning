//
// Created by sven on 2017-11-30.
//

#ifndef PATH_PLANNING_STAYINLANE_H
#define PATH_PLANNING_STAYINLANE_H

#include "VehicleBehavior.h"

class StayInLane2: public VehicleBehavior {
public:
    VehicleBehavior* nextBehaviorAndCalculateNextPath(vehicleAndMapState &state);
};


#endif //PATH_PLANNING_STAYINLANE_H

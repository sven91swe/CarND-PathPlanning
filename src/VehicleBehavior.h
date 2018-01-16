//
// Created by sven on 2017-11-30.
//

#ifndef PATH_PLANNING_VEHICLEBEHAVIOR_H
#define PATH_PLANNING_VEHICLEBEHAVIOR_H

#include <vector>
#include <iostream>
#include "support.h"

using namespace std;

class VehicleBehavior {
public:
    virtual VehicleBehavior* nextBehaviorAndCalculateNextPath(vehicleAndMapState &state, bool firstCall = true) = 0;
    vector<double> getNextX() {return nextX;}
    vector<double> getNextY() {return nextY;}
    void addLinkedBehvaior(VehicleBehavior* b) {linkedBehaviors.push_back(b);}
    virtual bool possibleToChangeToBehavior(vehicleAndMapState &state) {return true;}

protected:
    vector<double> nextX, nextY;
    vector<VehicleBehavior*> linkedBehaviors;
    double max_s = 6945.554;
};


#endif //PATH_PLANNING_VEHICLEBEHAVIOR_H

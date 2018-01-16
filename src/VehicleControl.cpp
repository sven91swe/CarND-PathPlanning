//
// Created by sven on 2017-11-29.
//

#include "VehicleControl.h"
#include "VehicleBehavior.h"
#include "StayInLane.h"
#include "ChangeLane.h"

void VehicleControl::init() {
  VehicleBehavior* stayInCurrentLane = new StayInLane;
  VehicleBehavior* changeLeft = new ChangeLane(-1);
  VehicleBehavior* changeRight = new ChangeLane(1);

  stayInCurrentLane->addLinkedBehvaior(changeLeft);
  stayInCurrentLane->addLinkedBehvaior(changeRight);

  changeLeft->addLinkedBehvaior(stayInCurrentLane);

  changeRight->addLinkedBehvaior(stayInCurrentLane);

  currentBehavior = stayInCurrentLane;
}

void VehicleControl::calculateNextPath(vehicleAndMapState &state)
{
  currentBehavior = currentBehavior->nextBehaviorAndCalculateNextPath(state);
}

vector<double> VehicleControl::getNextX() {
  return currentBehavior->getNextX();
}
vector<double> VehicleControl::getNextY() {
  return currentBehavior->getNextY();
}
//
// Created by sven on 2017-11-30.
//

#include <iostream>
#include "StayInLane.h"
#include "support.h"

VehicleBehavior* StayInLane2::nextBehaviorAndCalculateNextPath(vehicleAndMapState &state){
  double T = 0.5;

  vector<double> wantedSFinal = {state.car_s + T * mph2mps(48), mph2mps(48), 0};
  vector<double> wantedDFinal = {6, 0, 0};

  vector<vector<double>> initialSD = initialStateEstimationSD(state);

  vector<vector<double>> statesXY = stateInXY(initialSD.data()[0],
                                              initialSD.data()[1],
                                              wantedSFinal,
                                              wantedDFinal,
                                              state.car_yaw,
                                              *state.map_waypoints_s,
                                              *state.map_waypoints_x,
                                              *state.map_waypoints_y);

  vector<vector<double>> pathInXY = jerkMinimizedPath(statesXY.data()[0],
                                                      statesXY.data()[2],
                                                      statesXY.data()[1],
                                                      statesXY.data()[3],
                                                      T);

  //cout << "Speed: " << mph2mps(state.car_speed) << " - Estimation: " << initialSD.data()[0].data()[1] << " Diff: " << mph2mps(state.car_speed) - initialSD.data()[0].data()[1] << endl;
  //cout << "D: " << state.car_d << endl;

  vector<double> temp;
  nextX.clear();
  nextY.clear();

  for(double t=0.02; t<3; t+=0.02){
    temp = getXY(state.car_s + t * mph2mps(48), 6, *state.map_waypoints_s, *state.map_waypoints_x, *state.map_waypoints_y);
    nextX.push_back(temp[0]);
    nextY.push_back(temp[1]);
  }

  nextX = pathInXY.data()[0];
  nextY = pathInXY.data()[1];


  return this;
}


VehicleBehavior* oldNextBehaviorAndCalculateNextPath(vehicleAndMapState &state){
  vector<double> nextX;
  vector<double> nextY;
  vector<double> temp;
  for(double t=0.02; t<3; t+=0.02){
    temp = getXY(state.car_s + t * mph2mps(48), state.car_d, *state.map_waypoints_s, *state.map_waypoints_x, *state.map_waypoints_y);
    nextX.push_back(temp[0]);
    nextY.push_back(temp[1]);
  }

  return this;
}

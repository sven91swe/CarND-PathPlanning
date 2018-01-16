//
// Created by sven on 2017-11-30.
//

#include "ChangeLane.h"
#include "support.h"
#include "spline.h"

#include <iostream>
#include <cmath>


VehicleBehavior* ChangeLane::nextBehaviorAndCalculateNextPath(vehicleAndMapState &state, bool firstCall) {

  if(firstCall && state.previous_path_y->size() <= 25){
    if(linkedBehaviors.at(0)->possibleToChangeToBehavior(state)) {
      linkedBehaviors.at(0)->nextBehaviorAndCalculateNextPath(state, false);
      return linkedBehaviors.at(0);
    }
  }

  if(state.previous_path_x->size() > 25 && firstCall){
    nextX = *state.previous_path_x;
    nextY = *state.previous_path_y;
  }else {
    vector<vector<double>> XY = this->possiblePath(state, state.car_speed);
    nextX = XY.data()[0];
    nextY = XY.data()[1];
  }

  return this;
}

ChangeLane::ChangeLane(int directionIn)
  :VehicleBehavior(){
        direction = directionIn;
}

bool ChangeLane::possibleToChangeToBehavior(vehicleAndMapState &state) {
  int currentLane = laneNumber(state.car_d);
  int nextLane = currentLane + direction;

  bool carInFront = false;
  double middleOCurrentfLane = middleOfLane(currentLane);
  double other_S, other_D, deltaS;


  for(int i=0; i<state.sensor_fusion->size(); i++){
    other_S = state.sensor_fusion->at(i).at(5);
    other_D = state.sensor_fusion->at(i).at(6);

    if(abs(other_D-middleOCurrentfLane)<2){
      deltaS = other_S - state.car_s;
      if(0 <= deltaS && deltaS < mph2mps(state.car_speed)*5){
        carInFront = true;
      }
    }
  }

  if(!carInFront){
    return false;
  }

  double middleOfNextLane = middleOfLane(nextLane);
  if(middleOfNextLane > 11 || middleOfNextLane < 1){
    //cout << "Lane changed avoided due to being on the edge";
    return false;
  }


  for(int i=0; i<state.sensor_fusion->size(); i++){
    other_S = state.sensor_fusion->at(i).at(5);
    other_D = state.sensor_fusion->at(i).at(6);

    if(abs(other_D-middleOfNextLane)<2){
      deltaS = other_S - state.car_s;
      //cout << "deltaS: " << deltaS << " - direction: " << direction << endl;
      if(-1*max(mph2mps(50-state.car_speed), 5.0) < deltaS && deltaS < mph2mps(state.car_speed)*2.5){
        return false;
      }
    }
  }
  //cout << "PossibleToChangeLane - direction: " << direction << endl;
  return true;
}

vector<vector<double>> ChangeLane::possiblePath(vehicleAndMapState &state, double finalV_mph){

  vector<double>* previousX = state.previous_path_x;
  vector<double>* previousY = state.previous_path_y;

  double t=0.02;
  double max_t = 2;
  double extra_t = 1;
  double delta_t = 1;

  double tempX, tempY;

  double initialS, initialV, initialD, finalS, finalV, finalD, yaw_radian;

  initialS = state.car_s;
  initialV = state.car_speed;
  initialD = state.car_d;

  finalV = mph2mps(finalV_mph);
  initialV = mph2mps(state.car_speed);

  finalD = middleOfLane(laneNumber(state.car_d) + direction);

  if(finalD > 13 || finalD < 1){
    finalD = middleOfLane(laneNumber(state.car_d));
  }

  finalS = initialS + max_t*(initialV + finalV)/2;

  yaw_radian = deg2rad(state.car_yaw);


  vector<double> S, D, T;
  vector<double> X, Y;
  vector<vector<double>> temp;

  X.clear();
  Y.clear();

  double temp_X;
  double temp_Y;
  double speedAfterPrevious;
  double previousS;
  double previousD;

  if(previousX->size()>2){
    for(int i=0; i<previousX->size() && i<25; i++){
      temp_X = previousX->at(i);
      temp_Y = previousY->at(i);
      X.push_back(temp_X);
      Y.push_back(temp_Y);
      T.push_back(t);
      t+=0.02;
    }

    //TODO: This does probably cause some speed fluctiation.
    if(X.size()>11){
      speedAfterPrevious = sqrt(pow(X.at(X.size() - 1) - X.at(X.size() - 11), 2) +
                                pow(Y.at(Y.size() - 1) - Y.at(Y.size() - 11), 2)) / 0.2;
    }else {
      speedAfterPrevious = sqrt(pow(X.at(X.size() - 1) - X.at(X.size() - 2), 2) +
                                pow(Y.at(Y.size() - 1) - Y.at(Y.size() - 2), 2)) / 0.02;
    }
    vector<double> sd = getFrenet(X.back(), Y.back(), state.car_yaw, *state.map_waypoints_x, *state.map_waypoints_y);
    previousS = sd.at(0);
    previousD = sd.at(1);

  }else{
    S.push_back(initialS);
    D.push_back(initialD);
    T.push_back(0);
    previousS = initialS;
    previousD = initialD;

    speedAfterPrevious = mph2mps(state.car_speed);
  }

  S.push_back(previousS + speedAfterPrevious * (max_t-t));
  D.push_back(finalD);
  T.push_back(max_t);

  previousS += speedAfterPrevious * (max_t-t);

  S.push_back(previousS + speedAfterPrevious * extra_t/2);
  D.push_back(finalD);
  T.push_back(max_t + extra_t/2);

  S.push_back(previousS + speedAfterPrevious * extra_t);
  D.push_back(finalD);
  T.push_back(max_t + extra_t);


  temp = getXY(S, D, *state.map_waypoints_s, *state.map_waypoints_x, *state.map_waypoints_y);
  for(int i=0; i<temp.data()[0].size();i++){
    tempX = temp.data()[0].at(i);
    tempY = temp.data()[1].at(i);
    X.push_back(tempX);
    Y.push_back(tempY);
  }

  tk::spline splineX;
  tk::spline splineY;
  splineX.set_points(T, X);
  splineY.set_points(T, Y);

  vector<double> finalX, finalY;

  for(double t_iter=0.02; t_iter<max_t+extra_t; t_iter+=0.02) {
    finalX.push_back(splineX(t_iter));
    finalY.push_back(splineY(t_iter));
  }

  return {finalX, finalY};
}




//
// Created by sven on 2017-11-30.
//

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include "StayInLane.h"
#include "support.h"
#include "spline.h"

VehicleBehavior* StayInLane::nextBehaviorAndCalculateNextPath(vehicleAndMapState &state, bool firstCall){
  double T = 3;
/*
  if(exlusionArea(state.car_s)){
    cout << "ExlusionArea - S: " << state.car_s << endl;
    vector<vector<double>> pathInExlusionArea = this->possiblePath(state, 25);
    nextX = pathInExlusionArea.data()[0];
    nextY = pathInExlusionArea.data()[1];
    return this;
  }
*/
  possiblePathStruct best;
  pathEvaluation bestEval;
  best.score = - pow(10, 18);

  vector<possiblePathStruct> ListOfPaths;
  vector<double> speeds;

  for(double s=5; s<48.5; s+=0.25){
    speeds.push_back(s);
  }


  for(int i=0; i<speeds.size(); i++){
    possiblePathStruct temp;
    temp.final_speed_mph = speeds.at(i);
    temp.path = this->possiblePath2(state, temp.final_speed_mph);
    pathEvaluation resultEval = fitnessEstimatior(state, temp.path[0], temp.path[1]);
    temp.score = resultEval.score;
    ListOfPaths.push_back(temp);

    if(best.score < temp.score){
      best = temp;
      bestEval = resultEval;
    }
  }

    vector<double> posEstimate = getFrenet(state.car_x, state.car_y, deg2rad(state.car_yaw), *state.map_waypoints_x, *state.map_waypoints_y);
    double SEstimate = posEstimate.at(0);
    //cout << "S: " << state.car_s << " - S-diff: " << SEstimate - state.car_s << " - yaw: " << state.car_yaw << endl;


  cout << "Best is v= " << best.final_speed_mph << "\t - at S: " << state.car_s << "\t - deltaS: " << SEstimate - state.car_s << "\t - score: " << best.score << "\t - - " <<
       bestEval.distViolation << " " <<
       bestEval.vViolation << " " <<
       bestEval.aViolation << " " <<
       bestEval.jViolation << " " << endl;

  if(state.car_speed > 30){
    startUp = false;
  }else if(state.car_speed < 5){
    startUp = true;
  }

  if(best.final_speed_mph <= 43 && firstCall && !startUp && state.car_speed <= 43){
    for(int i=0; i<linkedBehaviors.size(); i++){
      if(linkedBehaviors.at(i)->possibleToChangeToBehavior(state)){
        linkedBehaviors.at(i)->nextBehaviorAndCalculateNextPath(state, false);
        vector<double> changeLaneX = linkedBehaviors.at(i)->getNextX();
        vector<double> changeLaneY = linkedBehaviors.at(i)->getNextY();

        pathEvaluation changeLaneEval = fitnessEstimatior(state, changeLaneX, changeLaneY);

        if(!(changeLaneEval.distViolation || changeLaneEval.vViolation || changeLaneEval.aViolation)) {
          return linkedBehaviors.at(i);
        }
      }
    }
  }




  //double finalVmph = 49;
  //vector<vector<double>> temp = this->possiblePath(state, finalVmph);
  //vector<vector<double>> temp = this->possiblePath2(state, finalVmph);
  //vector<vector<double>> temp = this->possiblePath3(state, finalVmph);




  nextX = best.path.data()[0];
  nextY = best.path.data()[1];

  return this;
}

vector<vector<double>> StayInLane::possiblePath(vehicleAndMapState &state, double finalV_mph) {

  vector<double>* previousX = state.previous_path_x;
  vector<double>* previousY = state.previous_path_y;

  double t=0.02;
  double max_t = 1.5;
  double extra_t = 0.2;
  double delta_t = 1;

  double tempX, tempY;

  double initialS, initialV, initialD, finalS, finalV, finalD, yaw_radian;

  initialS = state.car_s;
  initialV = state.car_speed;
  initialD = state.car_d;

  finalV = mph2mps(finalV_mph);
  initialV = mph2mps(state.car_speed);

  finalD = middleOfLane(laneNumber(state.car_d));

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
    for(int i=0; i<previousX->size() && i<20; i++){
      temp_X = previousX->at(i);
      temp_Y = previousY->at(i);
      X.push_back(temp_X);
      Y.push_back(temp_Y);
      T.push_back(t);
      t+=0.02;
    }

    //TODO: This does probably cause some speed fluctiation.
    if(X.size()>6){
      speedAfterPrevious = sqrt(pow(X.at(X.size() - 1) - X.at(X.size() - 6), 2) +
                                pow(Y.at(Y.size() - 1) - Y.at(Y.size() - 6), 2)) / 0.1;
    }else {
      speedAfterPrevious = sqrt(pow(X.at(X.size() - 1) - X.at(X.size() - 2), 2) +
                                pow(Y.at(Y.size() - 1) - Y.at(Y.size() - 2), 2)) / 0.02;
    }
    vector<double> sd = getFrenet(X.back(), Y.back(), deg2rad(state.car_yaw), *state.map_waypoints_x, *state.map_waypoints_y);
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


  double nextS, nextSpeed;
  double tempT;
  for(double iter_t = t+delta_t; iter_t <= t+3; iter_t+=delta_t){
    if(speedAfterPrevious < mph2mps(49)){
      if(speedAfterPrevious < finalV){
        //Acceleration
        nextSpeed = min(speedAfterPrevious + 3 * delta_t, finalV);
      }else{
        //Decceleration
        nextSpeed = max(speedAfterPrevious - 5 * delta_t, finalV);
      }
    }else{
      nextSpeed = finalV;
      speedAfterPrevious = finalV;
    }

    /*if(nextSpeed > finalV) {
      cout << "T: " << iter_t << " - nextSpeed: " << nextSpeed << " - speedAfterPrevious: " << speedAfterPrevious
           << endl;
    }*/

    S.push_back(previousS + (speedAfterPrevious + nextSpeed)/2 * delta_t);
    D.push_back(finalD);
    T.push_back(iter_t);

    previousS = previousS + (speedAfterPrevious + nextSpeed)/2 * delta_t;
    speedAfterPrevious = nextSpeed;
    tempT = iter_t;
  }
  t = tempT;

  S.push_back(previousS + speedAfterPrevious * extra_t);
  D.push_back(finalD);
  T.push_back(t + extra_t);


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

  for(double t_iter=0.02; t_iter<max_t; t_iter+=0.02) {
    finalX.push_back(splineX(t_iter));
    finalY.push_back(splineY(t_iter));
  }

  return {finalX, finalY};
}

vector<vector<double>> StayInLane::possiblePath2(vehicleAndMapState &state, double finalV_mph) {
  double extra_t = 0.5;
  double path_t = 2;
  double max_t = 1.5;

  vector<double> X, Y, T;

  vector<double>* previousX = state.previous_path_x;
  vector<double>* previousY = state.previous_path_y;

  int keepNumberOfSteps = 10;

  double t = 0.02;
  double temp_X, temp_Y;
  double initialSpeed;
  double S, D;

  if(previousX->size() > keepNumberOfSteps * 1.5){
    for(int i=0; i<keepNumberOfSteps;i++){
      temp_X = previousX->at(i);
      temp_Y = previousY->at(i);
      X.push_back(temp_X);
      Y.push_back(temp_Y);
      T.push_back(t);
      t += 0.02;
    }
    int deltaSteps = 2;
    dynamics pastPath = dynamicsEstimator(*previousX, *previousY, deltaSteps);

    double S_pos, D_pos;
    vector<double> SD = getFrenet(pastPath.pos.at(keepNumberOfSteps/deltaSteps).X, pastPath.pos.at(keepNumberOfSteps/deltaSteps).Y, deg2rad(state.car_yaw), *state.map_waypoints_x, *state.map_waypoints_y);

    S = SD.at(0);
    initialSpeed = pastPath.vel_abs.at(keepNumberOfSteps/deltaSteps);
  }else{
    X.push_back(state.car_x);
    Y.push_back(state.car_y);
    T.push_back(0);

    S = state.car_s;
    initialSpeed = state.car_speed;
  }

  double finalSpeed = mph2mps(finalV_mph);


  D = middleOfLane(laneNumber(state.car_d));
  S = S + (path_t - t) * (initialSpeed + finalSpeed)/2;

  vector<double> XY = getXY(S, D, *state.map_waypoints_s, *state.map_waypoints_x, *state.map_waypoints_y);
  temp_X = XY.at(0);
  temp_Y = XY.at(1);
  X.push_back(temp_X);
  Y.push_back(temp_Y);
  T.push_back(path_t);

  S = S + extra_t * finalSpeed;

  XY = getXY(S, D, *state.map_waypoints_s, *state.map_waypoints_x, *state.map_waypoints_y);
  temp_X = XY.at(0);
  temp_Y = XY.at(1);
  X.push_back(temp_X);
  Y.push_back(temp_Y);
  T.push_back(path_t + extra_t);


  tk::spline splineX;
  tk::spline splineY;
  splineX.set_points(T, X);
  splineY.set_points(T, Y);

  vector<double> finalX, finalY;

  for(double t=0.02; t<max_t; t+=0.02) {
    finalX.push_back(splineX(t));
    finalY.push_back(splineY(t));
  }

  return {finalX, finalY};
}

vector<vector<double>> StayInLane::possiblePath3(vehicleAndMapState &state, double finalV_mph){
  double t = 3;
  double extra_t = 0.2;
  double initialS, initialV, initialD, finalS, finalV, finalD;

  finalV = mph2mps(finalV_mph);
  initialS = state.car_s;
  initialV = mph2mps(state.car_speed);
  initialD = state.car_d;

  finalD = middleOfLane(laneNumber(state.car_d));

  finalS = initialS + t*(initialV + finalV)/2;

  vector<double> S(3), D(3), T(3);
  T[0] = 0; T[1] = t; T[2] = t + extra_t;
  S[0] = initialS; S[1] = finalS; S[2] = finalS + finalV * extra_t;
  D[0] = initialD; D[1] = finalD; D[2] = finalD;

  vector<double> X(3), Y(3);
  vector<vector<double>> temp;

  temp = getXY(S, D, *state.map_waypoints_s, *state.map_waypoints_x, *state.map_waypoints_y);

  X = temp.data()[0]; Y = temp.data()[1];

  tk::spline splineX;
  tk::spline splineY;
  splineX.set_points(T, X);
  splineY.set_points(T, Y);


  vector<double> finalX, finalY;

  for(double t=0.02; t<3; t+=0.02) {
    finalX.push_back(splineX(t));
    finalY.push_back(splineY(t));
  }

  return {finalX, finalY};
}



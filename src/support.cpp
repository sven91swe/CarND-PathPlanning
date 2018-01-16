//
// Created by sven on 2017-11-29.
//

#include <math.h>
#include <iostream>
#include <map>
#include "support.h"
#include "Eigen-3.3/Eigen/Dense"

using Eigen::MatrixXd;


constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

double mph2mps(double mph) { return mph * 1609.344 / 3600; }

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  return closestWaypoint;


  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size())
    {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{

  s = std::fmod(s, 6945.554);

  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}


int laneNumber(double d) {
  return d/4;
}

double middleOfLane(int laneNumber) {
  return laneNumber*4 + 2;
}

vector<vector<double>> getXY(vector<double> s, vector<double> d, const vector<double> &maps_s, const vector<double> &maps_x,
      const vector<double> &maps_y);

vector<vector<double>>
getXY(vector<double> s, vector<double> d, const vector<double> &maps_s, const vector<double> &maps_x,
      const vector<double> &maps_y) {
  vector<double> X;
  vector<double> Y;

  vector<double> temp;

  for(int i=0; i<s.size(); i++){
    temp = getXY(s.data()[i], d.data()[i], maps_s, maps_x, maps_y);
    X.push_back(temp[0]);
    Y.push_back(temp[1]);
  }

  return {X,Y};
}

pathEvaluation fitnessEstimatior(vehicleAndMapState &state, const vector<double> &X_vec, const vector<double> &Y_vec) {

  if(X_vec.size() != Y_vec.size()){
    perror("X and Y not of same size");
  }

  double sum=0;
  int limit = pow(10,9);
  double t=0.02;
  double delta_t = 0.02;

  double X, Y;
  double other_x, other_y, other_dx, other_dy, dist;


  double dist_point = 0;
  double v_point = 0;
  double a_point = 0;
  double j_point = 0;

  bool distViolation = false;
  bool vViolation = false;
  bool aViolation = false;
  bool jViolation = false;


  int dynamicsInterval = 5;
  dynamics vehicleDynamicsData = dynamicsEstimator(X_vec, Y_vec, dynamicsInterval);

  int dynamicsSearchLimit = vehicleDynamicsData.vel_abs.size();

  double v;
  for(int i=0; i<dynamicsSearchLimit; i++){
    v = vehicleDynamicsData.vel_abs.at(i);
    if(v > mph2mps(48) && !vViolation){
      v_point -= pow(10,9);
      vViolation = true;
    }else{
      v_point += v;
    }
  }


  double a;
  for(int i=0; i<dynamicsSearchLimit; i++){
    a = vehicleDynamicsData.acc_abs.at(i);
    if(a > 8 && !aViolation){
      a_point -= pow(10,6);
      aViolation = true;
    }else{
      a_point += (10 - a)*0.01;
    }
  }

  double j;
  for(int i=0; i<dynamicsSearchLimit; i++){
    j = vehicleDynamicsData.jerk_abs.at(i);
    if(j > 8 && !jViolation){
      j_point -= pow(10,3);
      jViolation = true;
    }else{
      j_point += (10 - j)*0.01;
    }
  }


  for(int i=0; i<X_vec.size() && i<limit; i++) {
    X = X_vec.at(i);
    Y = Y_vec.at(i);
    //Distance to other cars
    for (int j = 0; j < state.sensor_fusion->size(); j++) {
      other_x = state.sensor_fusion->at(j).at(1);
      other_y = state.sensor_fusion->at(j).at(2);
      other_dx = state.sensor_fusion->at(j).at(3);
      other_dy = state.sensor_fusion->at(j).at(4);

      dist = sqrt(pow(X - other_x, 2) + pow(Y - other_y, 2));

      if (dist < 1.5 && !distViolation) {
        dist_point -= pow(10, 12) * 100.0/i * vehicleDynamicsData.meanV; //To lower speed in event that many speeds have a potential collision.
        distViolation = true;
        /*cout << "Too close, dS: " << state.sensor_fusion->at(j).at(5) - state.car_s <<
             " - - dD: " << state.sensor_fusion->at(j).at(6) - state.car_d << endl;*/
      }
    }
  }


  //sum = dist_point + v_point + a_point + j_point;
  sum = dist_point + v_point + a_point;


  pathEvaluation toReturn;
  toReturn.score = sum;
  toReturn.vehicleDynamics;
  toReturn.distViolation = distViolation;
  toReturn.vViolation = vViolation;
  toReturn.aViolation = aViolation;
  toReturn.jViolation = jViolation;

  return toReturn;
}

dynamics dynamicsEstimator(const vector<double> &X_vec, const vector<double> &Y_vec, int deltaIndex){
  double X, Y, vX, vY, aX, aY, jX, jY;
  double old_X, old_Y, old_vX, old_vY, old_aX, old_aY;
  double other_x, other_y, other_dx, other_dy, dist;
  double v, a, j;

  double delta_t = 0.02 * deltaIndex;

  dynamics result;

  result.pos.push_back({X_vec.at(0), Y_vec.at(0)});
  result.vel.push_back({0, 0});
  result.acc.push_back({0, 0});
  result.jerk.push_back({0, 0});

  result.vel_abs.push_back(0);
  result.acc_abs.push_back(0);
  result.jerk_abs.push_back(0);

  for(int i=deltaIndex; i<X_vec.size() - deltaIndex; i+= deltaIndex){
    result.pos.push_back({X_vec.at(i), Y_vec.at(i)});
    vX = (X_vec.at(i + deltaIndex) - X_vec.at(i - deltaIndex))/(2*delta_t);
    vY = (Y_vec.at(i + deltaIndex) - Y_vec.at(i - deltaIndex))/(2*delta_t);
    v = sqrt(pow(vX, 2) + pow(vY, 2));
    result.vel.push_back({vX, vY});
    result.vel_abs.push_back(v);

    aX = (X_vec.at(i + deltaIndex) - 2*X_vec.at(i) + X_vec.at(i - deltaIndex))/pow(delta_t, 2);
    aY = (Y_vec.at(i + deltaIndex) - 2*Y_vec.at(i) + Y_vec.at(i - deltaIndex))/pow(delta_t, 2);
    a = sqrt(pow(aX, 2) + pow(aY, 2));
    result.acc.push_back({aX, aY});
    result.acc_abs.push_back(a);

    result.jerk.push_back({0, 0});
    result.jerk_abs.push_back(0);
  }

  double sumV = 0;
  for(int i=0; i<result.vel_abs.size(); i++){
    sumV += result.vel_abs.at(i);
  }

  result.meanV = sumV / result.vel_abs.size();

  return result;
}


bool exlusionArea(double S){
  return S > 6850 || S < 700;
}


dynamics dynamicsEstimator2(const vector<double> &X_vec, const vector<double> &Y_vec, int deltaIndex){
  double X, Y, vX, vY, aX, aY, jX, jY;
  double old_X, old_Y, old_vX, old_vY, old_aX, old_aY;
  double other_x, other_y, other_dx, other_dy, dist;
  double v, a, j;

  double delta_t = 0.02 * deltaIndex;

  dynamics result;

  //Initial V
  old_X = X_vec.at(0);
  old_Y = Y_vec.at(0);
  X = X_vec.at(deltaIndex);
  Y = Y_vec.at(deltaIndex);
  vX = (X - old_X)/delta_t;
  vY = (Y - old_Y)/delta_t;

  //Initial A
  old_X = X;
  old_Y = Y;
  old_vX = vX;
  old_vY = vY;
  X = X_vec.at(2 * deltaIndex);
  Y = Y_vec.at(2 * deltaIndex);
  vX = (X - old_X)/delta_t;
  vY = (Y - old_Y)/delta_t;
  aX = (vX - old_vX)/delta_t;
  aY = (vY - old_vY)/delta_t;

  for(int i=3 * deltaIndex; i<X_vec.size(); i+= deltaIndex) {
    old_X = X;
    old_Y = Y;
    old_vX = vX;
    old_vY = vY;
    old_aX = aX;
    old_aY = aY;
    X = X_vec.at(i);
    Y = Y_vec.at(i);
    vX = (X - old_X) / delta_t;
    vY = (Y - old_Y) / delta_t;
    aX = (vX - old_vX) / delta_t;
    aY = (vY - old_vY) / delta_t;
    jX = (aX - old_aX) / delta_t;
    jY = (aY - old_aY) / delta_t;

    result.pos.push_back({X, Y});
    result.vel.push_back({vX, vY});
    result.acc.push_back({aX, aY});
    result.jerk.push_back({jX, jY});

    v = sqrt(pow(vX, 2) + pow(vY, 2));
    a = sqrt(pow(aX, 2) + pow(aY, 2));
    j = sqrt(pow(jX, 2) + pow(jY, 2));

    v = abs(v);
    a = abs(a);
    j = abs(j);

    result.vel_abs.push_back(v);
    result.acc_abs.push_back(a);
    result.jerk_abs.push_back(j);
  }

  return result;
}




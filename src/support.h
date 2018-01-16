//
// Created by sven on 2017-11-29.
//

#ifndef PATH_PLANNING_SUPPORT_H
#define PATH_PLANNING_SUPPORT_H

#include <vector>

using namespace std;

struct vehicleAndMapState {
    double car_x, car_y, car_s, car_d, car_yaw, car_speed;
    vector<double>* previous_path_x;
    vector<double>* previous_path_y;
    double end_path_s, end_path_d;
    vector<vector<double>>* sensor_fusion;
    vector<double>* map_waypoints_x;
    vector<double>* map_waypoints_y;
    vector<double>* map_waypoints_s;
    vector<double>* map_maypoints_dx;
    vector<double>* map_waypoints_dy;
};

struct possiblePathStruct{
    vector<vector<double>> path;
    double score;
    double final_speed_mph;
};

struct XY{
    double X;
    double Y;
};

struct dynamics{
    vector<XY> pos;
    vector<XY> vel;
    vector<double> vel_abs;
    vector<XY> acc;
    vector<double> acc_abs;
    vector<XY> jerk;
    vector<double> jerk_abs;
    double meanV;
};

struct pathEvaluation{
    dynamics vehicleDynamics;
    double score;
    bool distViolation = false;
    bool vViolation = false;
    bool aViolation = false;
    bool jViolation = false;
};

constexpr double pi();

double deg2rad(double x);

double rad2deg(double x);

double mph2mps(double mph);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

vector<vector<double>> getXY(vector<double> s, vector<double> d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);


int laneNumber(double d);

double middleOfLane(int laneNumber);

pathEvaluation fitnessEstimatior(vehicleAndMapState &state, const vector<double> &X, const vector<double> &Y);

dynamics dynamicsEstimator(const vector<double> &X_vec, const vector<double> &Y_vec, int deltaIndex);

bool exlusionArea(double S);




#endif //PATH_PLANNING_SUPPORT_H

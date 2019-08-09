#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include "trajectory.h"
#include "behaviour.h"
#include "jmt.h"
#include "vehicle.h"
#include "helpers.h"
#include "mapping.h"
#include "constants.h"
#include "spline.h"

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::sort;

Trajectory::Trajectory(){}

Trajectory::Trajectory(vector<double> x, vector<double> y, vector<double> s, vector<double> s_d, vector<double> s_dd,
                        vector<double> d, vector<double> d_d, vector<double> d_dd, vector<double> yaw, vector<double> target){
    this->x = x;
    this->y = y;
    this->s = s;
    this->s_d = s_d;
    this->s_dd = s_dd;
    this->d = d;
    this->d_d = d_d;
    this->d_dd = d_dd;
    this->yaw = yaw;
    this->target = target;
}

//double Trajectory::Trajectory cost(State state, vector<Vehicle> predictions,double T)

Trajectory::~Trajectory(){}
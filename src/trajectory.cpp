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

Trajectory::Trajectory(double x, double y, double s, double s_d, double s_dd, double d, double d_d, double d_dd, double yaw){
    this->x = x;
    this->y = y;
    this->s = s;
    this->s_d = s_d;
    this->s_dd = s_dd;
    this->d = d;
    this->d_d = d_d;
    this->d_dd = d_dd;
    this->yaw = yaw;
}

Trajectory Trajectory::generate(string state, vector<Vehicle> predictions, double T){
    
}

Trajectory::~Trajectory(){}
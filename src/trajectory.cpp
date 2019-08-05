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

Trajectory::Trajectory(Vehicle* vehicle, double s_d, double s_dd, double d_d, double d_dd){
    this->vehicle = vehicle;
    this->s_d = s_d;
    this->s_dd = s_dd;
    this->d_d = d_d;
    this->d_dd = d_dd;
}

Trajectory::~Trajectory(){}
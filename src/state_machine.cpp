#include <iostream>
#include <math.h>
#include <string>
#include <vector>
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

vector<string> const STATES = {"READY", "KL", "PLCL", "PLCR", "LCL", "LCR"};

State::State(){}

State::State(double s, double d, int current_lane, int intended_lane){
    this->s = s;
    this->d = d;
    this->current_lane = current_lane;
    this->intended_lane = intended_lane;
}

StateMachine::StateMachine()
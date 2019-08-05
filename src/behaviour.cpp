#include <iostream>
#include <math.h>
#include <string>
#include <vector>
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

Behaviour::Behaviour(Vehicle ego, double ref_vel){
    this->s = ego.s;
    this->d = ego.d;
    this->lane = ego.lane;
    this->current_state = ego.state;
    this->ref_vel = ref_vel;
}

vector<string> Behaviour::available_states(){
    vector<string> states;
    if (this->current_state == "KL"){
        states = {"KL", "PLCL", "PLCR"};
    }
    else if (this->current_state == "PLCL"){
        if (this->lane > 0){
            states = {"KL", "PLCL","LCL"};
        }
        else if (this->lane == 0){
            states = {"KL", "PLCR"};
        }
    }
    else if (this->current_state == "PLCR"){
        if (this->lane < 2){
            states = {"KL", "PLCR", "LCR"};
        }
        else if (this->lane == 2){
            states = {"KL", "PLCL"};
        }
    }
    else if ((this->current_state == "LCL") || (this->current_state == "LCR")){
        states = {"KL"};
    }

    return states;
}

/*
vector<Trajectory> Behaviour::generate_trajectory(vector<Vehicle> predictions){

}

 */





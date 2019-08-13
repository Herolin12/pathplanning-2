#include <iostream>
#include <algorithm>
#include <map>
#include <math.h>
#include <string>
#include <vector>
#include "behaviour.h"
#include "trajectory.h"
#include "vehicle.h"
#include "mapping.h"
#include "cost_functions.h"
#include "jmt.h"
#include "helpers.h"
#include "constants.h"
#include "spline.h"

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::endl;
using std::sort;
using std::min_element;
using std::map;

State::State(){}

State::State(string id, int lane){
    this->id = id;
    this->current_lane = lane;
    if ((this->id == "KL") || (this->id == "RDY")){
        this->intended_lane = this->current_lane;
        this->time_ahead = 1;
    }
    else if ((this->id != "PLCL") || (this->id != "PLCR")){
        this->time_ahead = 1;
        if (this->id == "PLCL"){
            this->intended_lane = this->current_lane - 1;
        }
        else if (this->id == "PLCR"){
            this->intended_lane = this->current_lane + 1;
        }
    }
    else if (this->id == "LCL"){
        this->intended_lane = this->current_lane - 1;
        this->time_ahead = MAX_LANE_CHANGE_TIME;
    }
    else if (this->id == "LCR"){
        this->intended_lane = this->current_lane + 1;
        this->time_ahead = MAX_LANE_CHANGE_TIME;
    }
}

State::~State(){}

Behaviour::Behaviour(){}

Behaviour::Behaviour(Vehicle* ego, double ref_vel){
    this->ego = ego;
    this->ref_vel = ref_vel;
    this->current_timestep = 0;
    vector<State> states;
    vector<string> states_id;
    states.push_back(State("RDY", this->ego->lane));
    states.push_back(State("KL", this->ego->lane));
    states.push_back(State("PLCL", this->ego->lane));
    states.push_back(State("PLCR", this->ego->lane));
    states.push_back(State("LCL", this->ego->lane));
    states.push_back(State("LCR", this->ego->lane));
    this->states = states;
}

// states machine
vector<State> Behaviour::available_states(){
    vector<State> states;
    State current_state = *this->ego->state;
    if ((current_state.id == "KL") || (current_state.id == "RDY")){
        states = {this->states[1], this->states[2], this->states[3]};
    }
    else if (current_state.id == "PLCL"){
        if (this->ego->lane > 0){ states = {this->states[1], this->states[2], this->states[4]}; }
        else if (this->ego->lane == 0){ states = {this->states[1], this->states[3]}; }
    }
    else if (current_state.id == "PLCR"){
        if (this->ego->lane < 2){ states = {this->states[1], this->states[3], this->states[5]}; }
        else if (this->ego->lane == 2){ states = {this->states[1], states[2]}; }
    }
    else if ((current_state.id == "LCL") || (current_state.id == "LCR")){
        states = {this->states[1], this->states[2], this->states[3]};
    }
    return states;
}

vector<Trajectory> Behaviour::generate_trajectory(vector<double> x, vector<double> y){
    // To get available states of current state
    vector<State> states = available_states();
    vector<Trajectory> trajectories;
    Trajectory trajectory;
    // start conditions
    vector<double> k = this->ego->kinematics;
    vector<double> start = {k[0], k[1], k[2], k[3], k[4], k[5]};
    // To generate max number of trajectories up to 1 using different acceleration
    vector<double> different_acc = {MAX_ACCELERATION};
    // Making kinematics of ego
    vector<double> ego_kinematics = {start[0], start[1], start[2]};
    vector<double> target, target_s;
    vector<vector<double>> kinematics_s, kinematics_d;

    for (State& state: states){
        // Intended lane of the state
        double intended_lane = state.intended_lane;
        double time_to_complete = (state.time_ahead*50 - x.size())*PATH_TIMESTEP;
        for (double& acc: different_acc){
            // To estimate future/ final kinematics of ego if it travels with this certain acceleration
            target_s = calculate_final_kinematics(ego_kinematics, acc, time_to_complete);
            // Target conditions
            target = {target_s[0], target_s[1], target_s[2], intended_lane, 0, 0};
            // Create new trajectory object
            trajectory = Trajectory(&state, this->ego, start, target, x, y, kinematics_s, kinematics_d, time_to_complete);
            trajectory.generate();
            trajectories.push_back(trajectory);
        }
    }
    return trajectories;
}

Trajectory Behaviour::get_best_trajectory(vector<Trajectory> trajectories, vector<Vehicle> predictions){
    // Get sensor fusions and classify them into cars ahead, behind, left or right
    vector<vector<Vehicle>> surroundings;
    float cost;
    vector<float> traj_cost;
    for (Trajectory& traj: trajectories){
        surroundings = prepare_cost(this->ego, predictions, traj.time_to_complete);
        cost = traj.cost(surroundings);
        traj_cost.push_back(cost);
    }
    int best = min_element(traj_cost.begin(), traj_cost.end()) - traj_cost.begin();
    cout << "best: " << best << endl;
    return trajectories[best];
}

Behaviour::~Behaviour(){}
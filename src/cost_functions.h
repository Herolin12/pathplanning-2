#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include "trajectory.h"
#include "behaviour.h"
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

vector<vector<Vehicle>> cost_preparation(Vehicle ego, vector<Vehicle> const others){
    vector<vector<Vehicle>> surroundings;
    vector<Vehicle> vehicles_ahead = ego.ahead(others);
    vector<Vehicle> vehicles_behind = ego.behind(others);

    surroundings = {vehicles_ahead, vehicles_behind};

    return surroundings; 
}

float costfunc_Acceleration(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    vector<Vehicle> vehicles_ahead = surroundings[0];
    double cost;
    if (vehicles_ahead.empty()){
        if (ego.speed <= MAX_SPEED_M){
            cost = 0;
        }
    }
}

float costfunc_Collision(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    int start_lane = state.current_lane;
    int final_lane = state.intended_lane;
    vector<Vehicle> vehicles_ahead = surroundings[0];
    vector<Vehicle> vehicles_behind = surroundings[1];
    double cost = 0;

    float cost;
    if (state.id == "KL"){
        if (!vehicles_ahead.empty()){
            cost += 1;
        }
    }
    else if (state.id != "KL"){
        cost += 0.1;
    }
}

float costfunc_Feasibility(State state, Vehicle ego, vector<vector<Vehicle>> const others, double T, double weight){
    int start_lane = state.current_lane;
    int final_lane = state.intended_lane;


}
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include "trajectory.h"
#include "helpers.h"
#include "behaviour.h"
#include "vehicle.h"
#include "mapping.h"
#include "constants.h"
#include "spline.h"

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::sort;

vector<vector<Vehicle>> prepare_cost(Vehicle ego, vector<Vehicle> const others){
    vector<Vehicle> vehicles_ahead = ego.ahead(others);
    vector<Vehicle> vehicles_behind = ego.behind(others);
    vector<Vehicle> vehicles_left = ego.side(others, 'L');
    vector<Vehicle> vehicles_right = ego.side(others, 'R');
    return {vehicles_ahead, vehicles_behind, vehicles_left, vehicles_right};
}

float costfunc_Speed(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // cost function for maintaining highest speed/ speed limit possible
    float target_speed = MAX_SPEED_MS;
    float cost;
    if (ego.speed > target_speed){
        cost = 1;
    }
    else if (ego.speed <= target_speed){
        cost = 1 - (ego.speed/ target_speed);
    }
    return cost;
}

float costfunc_BufferDistance(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // cost function for buffering a distance with other surroundings vehicles
    double buffer = BUFFER_RANGE;
    vector<Vehicle> ahead = surroundings[0];
    vector<Vehicle> behind = surroundings[1];
    float cost_1, cost_2, cost;

    if (!ahead.empty()){
        cost_1 = exp(-((ahead[0].s - ego.s)/ buffer));
    }
    else if (ahead.empty()){
        cost_1 = 0;
    }

    if (!behind.empty()){
        cost_2 = exp(-((ego.s - behind[0].s)/ buffer));
    }
    else if (behind.empty()){
        cost_2 = 0;
    }

    cost = (cost_1 + cost_2)/2;
    return cost;
}

float costfunc_LaneChange(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // Cost function for lane changing
    int start_lane = state.current_lane;
    int final_lane = state.intended_lane;
    float current_speed = ego.speed;
    float new_speed = lane_speed(surroundings, final_lane);
    float cost;
    float distance_to_goal = abs(TRACK_LENGTH - ego.s);

    // TODO: How long the lane change takes, must be less or equals to 3s

}

float lane_speed(vector<vector<Vehicle>> const surroundings, int lane){
    // To get driving speed in selected lane
    float speed;
    for (int i = 0; i < surroundings.size(); i++){
        // If found vehicle in the lane
        if (surroundings[i][0].lane == lane){
            // Only get the speed of closest vehicle
            speed = surroundings[i][0].speed;
            return speed;
        }
        else {
            continue;
        }
    }
    // If found no vehicle in the lane
    return MAX_SPEED_MS;
}

float costfunc_SpeedChange(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // Penalize negative speed change
    int start_lane = state.current_lane;
    int final_lane = state.intended_lane;
    float current_speed = ego.speed;
    float new_speed = lane_speed(surroundings, final_lane);
    float speed_change = (new_speed - current_speed)/ current_speed;
    float cost;

    cost = 1 - sigmoid(speed_change);
    return cost;
}

// TODO: cost functions for acceleration and jerk

float costfunc_LongitudinalCollision(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    int start_lane = state.current_lane;
    int final_lane = state.intended_lane;
    double buffer = BUFFER_RANGE;
    vector<Vehicle> vehicles_ahead = surroundings[0];
    vector<Vehicle> vehicles_behind = surroundings[1];
    float cost_1, cost_2, cost, distance;
    // distance, speed, predicted position
    Vehicle next_ego = ego.predict_position(T);
    if (!vehicles_ahead.empty()){
        Vehicle predicted = vehicles_ahead[0].predict_position(T);
        cost_1 = exp(-((predicted.s - next_ego.s)/ buffer));
    }
    else if (vehicles_ahead.empty()){
        cost_1 = 0;
    }

    if (!vehicles_behind.empty()){
        Vehicle predicted = vehicles_behind[0].predict_position(T);
        cost_2 = exp(-((next_ego.s - predicted.s)/ buffer));
    }
    else if (vehicles_behind.empty()){
        cost_2 = 0;
    }

    cost = (cost_1 + cost_2)/2;
    return cost;
}

float costfunc_LatitudinalCollision(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight, char mode){
    int start_lane = state.current_lane;
    int final_lane = state.intended_lane;
    double buffer = BUFFER_RANGE*1.2;
    vector<Vehicle> vehicles_left = surroundings[2];
    vector<Vehicle> vehicles_right = surroundings[3];
    float total_distance = 0;
    int vehicles_count = 0;
    float cost;

    // distance, speed, predicted position
    Vehicle next_ego = ego.predict_position(T);
    if (mode == 'L'){
        if (!vehicles_left.empty()){
            for (Vehicle &v: vehicles_left){
                Vehicle predicted = v.predict_position(T);
                double distance = get_distance(next_ego.x, next_ego.y, predicted.x, predicted.y);
                if (distance >= buffer){
                    vehicles_count++;
                    total_distance += distance;
                }
                else {
                    continue;
                }
            }
        }
        else if (vehicles_left.empty()){
            cost = 0;
            return cost;
        }
    }

    else if (mode == 'R'){
        if (!vehicles_right.empty()){
            for (Vehicle &v: vehicles_right){
                Vehicle predicted = v.predict_position(T);
                double distance = get_distance(next_ego.x, next_ego.y, predicted.x, predicted.y);
                if (distance >= buffer){
                    vehicles_count++;
                    total_distance += distance;
                }
                else {
                    continue;
                }
            }
        }
        else if (vehicles_right.empty()){
            cost = 0;
            return cost;
        }
    }
    cost = exp(-(total_distance/(vehicles_count*buffer)));
    return cost;
}

float costfunc_Collision(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    int start_lane = state.current_lane;
    int final_lane = state.intended_lane;
    vector<Vehicle> vehicles_ahead = surroundings[0];
    vector<Vehicle> vehicles_behind = surroundings[1];
    float lat_cost, long_cost, cost;
    char mode;
    double long_weight = 0;
    double lat_weight = 0;

    if (state.id == "KL"){
        lat_cost = 0;
        long_cost = costfunc_LongitudinalCollision(state, ego, surroundings, T, weight);
    }
    else if (state.id != "KL"){
        if (state.id.back() == 'L'){
            mode = 'L';
        }
        else if (state.id.back() == 'R'){
            mode = 'R';
        }

        lat_cost = costfunc_LatitudinalCollision(state, ego, surroundings, T, weight, mode);
        long_cost = costfunc_LongitudinalCollision(state, ego, surroundings, T, weight);
    }


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

float costfunc_Acceleration(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    vector<Vehicle> vehicles_ahead = surroundings[0];
    double cost;
    if (vehicles_ahead.empty()){
        if (ego.speed <= MAX_SPEED_M){
            cost = 0;
        }
    }
}

float costfunc_Jerk(State state, Vehicle ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    vector<Vehicle> vehicles_ahead = surroundings[0];
}

float costfunc_Feasibility(State state, Vehicle ego, vector<vector<Vehicle>> const others, double T, double weight){
    int start_lane = state.current_lane;
    int final_lane = state.intended_lane;
}

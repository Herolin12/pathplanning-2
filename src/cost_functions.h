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
using std::abs;
using std::cout;
using std::sort;

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

vector<vector<Vehicle>> prepare_cost(Vehicle* ego, vector<Vehicle> const others, double T){
    vector<Vehicle> vehicles_ahead = ego->ahead(others, T);
    vector<Vehicle> vehicles_behind = ego->behind(others, T);
    vector<Vehicle> vehicles_left = ego->side(others, T, 'L');
    vector<Vehicle> vehicles_right = ego->side(others, T, 'R');
    return {vehicles_ahead, vehicles_behind, vehicles_left, vehicles_right};
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Legality Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_Speed(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // cost function for maintaining highest speed/ speed limit possible
    float target_speed = MAX_SPEED_MS;
    float cost;
    double final_speed = trajectory->target[1];
    if (final_speed > target_speed){
        cost = 1;
    }
    else if (final_speed <= target_speed){
        cost = 1 - (final_speed/ target_speed);
    }
    return cost;
}

float subcost_BufferDistance(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // cost function for buffering a distance with other surroundings vehicles
    double buffer = BUFFER_RANGE;
    Vehicle* next_ego = new Vehicle(trajectory->ego->predict_position(T));
    double time_ahead = trajectory->time_to_complete; 
    vector<Vehicle> ahead = surroundings[0];
    vector<Vehicle> behind = surroundings[1];
    float cost_1, cost_2, cost;

    if (!ahead.empty()){
        cost_1 = exp(-((ahead[0].s - next_ego->s)/ buffer));
    }
    else if (ahead.empty()){
        cost_1 = 0;
    }

    if (!behind.empty()){
        cost_2 = exp(-((next_ego->s - behind[0].s)/ buffer));
    }
    else if (behind.empty()){
        cost_2 = 0;
    }

    cost = (cost_1 + cost_2)/2;
    return cost;
}

float costfunc_Legality(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    double cost_1 = subcost_Speed(trajectory, surroundings, T, weight);
    double cost_2 = subcost_BufferDistance(trajectory, surroundings, T, weight);
    double cost = (cost_1 + cost_2)/2;
    return cost;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Efficiency Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_LaneChange(Trajectory* trajectory, Vehicle* ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // Cost function for lane changing
    State* state = trajectory->state;
    int start_lane = state->current_lane;
    int intended_lane = state->intended_lane;
    float distance_to_goal = abs(TRACK_LENGTH - ego->s);

    float distance = 2*(start_lane - intended_lane);
    float exponent = exp(-(abs(distance)/ distance_to_goal));
    float cost = 1 - exponent;
    return cost;
    

    // TODO: How long the lane change takes, must be less or equals to 3s
}

float subcost_SpeedChange(Trajectory* trajectory, Vehicle* ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    // Penalize negative speed change
    State* state = trajectory->state;
    int start_lane = state->current_lane;
    int intended_lane = state->intended_lane;
    float target_speed  = MAX_SPEED_MS;
    float current_speed = ego->speed;
    float new_speed = lane_speed(surroundings, intended_lane);
    float speed_change = new_speed - current_speed;

    float cost = (2.0*target_speed - current_speed - new_speed)/ (2.0*target_speed);
    return cost;
}

float costfunc_Efficiency(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    Vehicle* next_ego = new Vehicle(trajectory->ego->predict_position(T));
    float cost_1 = subcost_LaneChange(trajectory, next_ego, surroundings, T, weight);
    float cost_2 = subcost_SpeedChange(trajectory, next_ego, surroundings, T, weight);
    float cost = (cost_1 + cost_2)/ 2;
    return cost;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TODO: cost functions for acceleration and jerk

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Safety Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_LongitudinalCollision(Trajectory* trajectory, Vehicle* ego, vector<vector<Vehicle>> const surroundings, double T, double weight){
    int start_lane = trajectory->state->current_lane;
    int intended_lane = trajectory->state->intended_lane;
    double buffer = BUFFER_RANGE;
    vector<Vehicle> vehicles_ahead = surroundings[0];
    vector<Vehicle> vehicles_behind = surroundings[1];
    float cost_1, cost_2, cost, distance;
    // distance, speed, predicted position
    if (!vehicles_ahead.empty()){
        cost_1 = exp(-((vehicles_ahead[0].s - ego->s)/ buffer));
    }
    else if (vehicles_ahead.empty()){
        cost_1 = 0;
    }

    if (!vehicles_behind.empty()){
        cost_2 = exp(-((ego->s - vehicles_ahead[0].s)/ buffer));
    }
    else if (vehicles_behind.empty()){
        cost_2 = 0;
    }

    cost = (cost_1 + cost_2)/2;
    return cost;
}

float subcost_LatitudinalCollision(Trajectory* trajectory, Vehicle* ego, vector<vector<Vehicle>> const surroundings, double T, double weight, char mode){
    int start_lane = trajectory->state->current_lane;
    int intended_lane = trajectory->state->intended_lane;
    double buffer = BUFFER_RANGE*1.2;
    vector<Vehicle> vehicles_left = surroundings[2];
    vector<Vehicle> vehicles_right = surroundings[3];
    float total_distance = 0;
    int vehicles_count = 0;
    float cost;

    // distance, speed, predicted position
    if (mode == 'L'){
        if (!vehicles_left.empty()){
            for (Vehicle &v: vehicles_left){
                double distance = get_distance(ego->x, ego->y, v.x, v.y);
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
                double distance = get_distance(ego->x, ego->y, v.x, v.y);
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

float costfunc_Collision(Trajectory* trajectory, vector<vector<Vehicle>> const surroundings, double T, double weight){
    Vehicle* next_ego = new Vehicle(trajectory->ego->predict_position(T));
    int start_lane = trajectory->state->current_lane;
    int intended_lane = trajectory->state->intended_lane;
    vector<Vehicle> vehicles_ahead = surroundings[0];
    vector<Vehicle> vehicles_behind = surroundings[1];
    float lat_cost, long_cost, cost;
    char mode;

    if (trajectory->state->id == "KL"){
        lat_cost = 0;
        long_cost = subcost_LongitudinalCollision(trajectory, next_ego, surroundings, T, weight);
        cost = long_cost;
    }
    else if (trajectory->state->id != "KL"){
        if (trajectory->state->id.back() == 'L'){ mode = 'L'; }
        else if (trajectory->state->id.back() == 'R'){ mode = 'R'; }
        lat_cost = subcost_LatitudinalCollision(trajectory, next_ego, surroundings, T, weight, mode);
        long_cost = subcost_LongitudinalCollision(trajectory, next_ego, surroundings, T, weight);
        cost = (lat_cost + long_cost)/ 2;
    }
    return cost;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                    /////     Comfort Cost      /////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
float subcost_Acceleration(State* state, Vehicle* ego, Trajectory *trajectory, double T, double weight){
    double max_acceleration = MAX_ACCELERATION;
    vector<double> k = ego->kinematics;
    vector<double> traj_acceleration = {k[2], k[5]};
    double acceleration = sqrt(pow(traj_acceleration[0], 2) + pow(traj_acceleration[1], 2));
    float cost;

    if ((acceleration >= max_acceleration) || (ego->speed >= MAX_SPEED_MS)){
        cost = 1;
    }
    else if (acceleration < max_acceleration){
        double acc_diff = max_acceleration - acceleration;
        cost = 1 - (acc_diff/ max_acceleration);
    }
    return cost;
}

float subcost_Jerk(State* state, Vehicle* ego, Trajectory* trajectory, double T, double weight){
    double max_jerk = MAX_JERK;
    float cost = 0;
    return cost;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
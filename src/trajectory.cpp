#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include "trajectory.h"
#include "vehicle.h"
#include "behaviour.h"
#include "jmt.h"
#include "helpers.h"
#include "constants.h"
#include "spline.h"

// for convenience
using std::string;
using std::vector;
using std::cout;
using std::sort;

Trajectory::Trajectory(){}

Trajectory::Trajectory(State* state, Vehicle* ego, vector<double> start, vector<double> target, vector<double> x, vector<double> y,
                        vector<vector<double>> kinematics_s, vector<vector<double>> kinematics_d, double time_to_complete){
    this->state = state;
    this->ego = ego;
    this->start = start;
    this->target = target;
    this->x = x;
    this->y = y;
    this->time_to_complete = time_to_complete;
}

void Trajectory::generate(){
    double T = this->state->time_ahead;
    this->jmt = get_jmt(this->start, this->target, T);
    vector<double> alpha_s = jmt[0];
    vector<double> alpha_d = jmt[1];
    vector<double> xy, next_s, next_d;
    double xt, yt;
    double t = 0;

    while (t < T){
        t += PATH_TIMESTEP;
        vector<vector<double>> next_waypoints = generation_next_waypoints(start, target, alpha_s, alpha_d, t);
        next_s = next_waypoints[0];
        next_d = next_waypoints[1];
        xy = this->ego->map->getXY(next_s[0], next_d[1]);
        xt = xy[0];
        yt = xy[1];
        this->x.push_back(xt);
        this->y.push_back(yt);
        this->kinematics_s.push_back(next_s);
        this->kinematics_d.push_back(next_d);
    }
}

float Trajectory::cost(vector<vector<Vehicle>> surroundings){
    float cost;
    float legality_cost = costfunc_Legality(this, surroundings, this->time_to_complete, LEGALITY_COST);
    float efficiency_cost = costfunc_Efficiency(this, surroundings, this->time_to_complete, EFFICIENCY_COST);

    cost = (legality_cost + efficiency_cost)/ 2;
    return cost;
}

Trajectory::~Trajectory(){}
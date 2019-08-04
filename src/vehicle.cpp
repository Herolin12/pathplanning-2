#include <iostream>
#include <math.h>
#include <utility>
#include <algorithm>
#include "vehicle.h"
#include "helpers.h"
#include "mapping.h"
#include "constants.h"

using std::sort;
using namespace std;

int get_lane(double d){
    double new_d = d/ LANE_WIDTH;
    int lane;

    if (new_d < 1){
        lane = 0;
    }
    else if ((new_d >= 1) && (new_d < 2)){
        lane = 1;
    }
    else if (new_d >= 2){
        lane = 2;
    }

    return lane;
}

struct sort_greater {
    bool operator()(const Vehicle& x, const Vehicle& y) {
        if (x.s != y.s) {
            return x.s < y.s;
        }
        return x.id < y.id;
    }
};

Vehicle::Vehicle() {}

Vehicle::Vehicle(int id, double x, double y, double vx, double vy, double s, double d, Mapping* map)
{
    this->id = id;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
    this->map = map;
    this->theta = get_theta(vx, vy);
    this->lane = get_lane(this->d);
    this->speed = get_speed();
}

double Vehicle::get_speed() const {
    return sqrt(this->vx * this->vx + this->vy * this->vy);
}

vector<Vehicle> Vehicle::ahead(vector<Vehicle> others) {
    vector<Vehicle> vehicles_ahead;
    for (Vehicle &v: others){
        if ((v.lane == this->lane) && (v.s >= this->s)){
            vehicles_ahead.push_back(v);
        }
        else {
            continue;
        }
    }

    if (vehicles_ahead.size() > 1) {
        sort(vehicles_ahead.begin(), vehicles_ahead.end(), sort_greater());
    }
    return vehicles_ahead;
}

vector<Vehicle> Vehicle::behind(vector<Vehicle> others) {
    vector<Vehicle> vehicles_behind;
    for (Vehicle &v: others){
        if ((v.lane == this->lane) && (v.s < this->s)){
            vehicles_behind.push_back(v);
        }
        else {
            continue;
        }
    }
    return vehicles_behind;
}

Vehicle Vehicle::predict_position(double t){
    double new_x = this->x + (this->vx * t);
    double new_y = this->y + (this->vy * t);
    double theta = atan2(new_x - this->y, new_y - this->x);
    vector<double> frenet = this->map->getFrenet(new_x, new_y, theta);
    return Vehicle(this->id, new_x, new_y, this->vx, this->vy, frenet[0], frenet[1], this->map);
}

Vehicle::~Vehicle() {}
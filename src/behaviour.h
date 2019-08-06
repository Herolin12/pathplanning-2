#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include "trajectory.h"
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

class State {
    public:
        string id;
        int current_lane;
        int intended_lane;

        State();
        State(string id, int current_lane, int intended_lane);

        virtual ~State();
};

class Behaviour {
    public:
        double s;
        double d;
        int lane;
        double ref_vel;
        State current_state;

        Behaviour(Vehicle ego, double ref_vel);

        virtual ~Behaviour();

        vector<string> available_states();
        vector<Trajectory> generate_trajectory(vector<Vehicle> predictions, double T);
};

#endif

#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <iostream>
#include <vector>
#include "vehicle.h"

class Trajectory {
    public:
        vector<double> x;
        vector<double> y;
        vector<double> s;
        vector<double> s_d;
        vector<double> s_dd;
        vector<double> d;
        vector<double> d_d;
        vector<double> d_dd;
        vector<double> yaw;
        vector<double> target;

        Trajectory();
        Trajectory(vector<double> x, vector<double> y, vector<double> s, vector<double> s_d, vector<double> s_dd,
                    vector<double> d, vector<double> d_d, vector<double> d_dd, vector<double> yaw, vector<double> target);

        virtual ~Trajectory();

};

#endif

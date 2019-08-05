#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <iostream>
#include <vector>
#include "vehicle.h"

class Trajectory
{
    public:
        double x;
        double y;
        double s;
        double s_d;
        double s_dd;
        double d;
        double d_d;
        double d_dd;
        double yaw;

        Trajectory();
        Trajectory(double x, double y, double s, double s_d, double s_dd, double d, double d_d, double d_dd, double yaw);

        virtual ~Trajectory();

};

#endif

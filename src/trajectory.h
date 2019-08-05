#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <iostream>
#include <vector>
#include "vehicle.h"

class Trajectory
{
    public:
        Vehicle* vehicle;
        double s_d;
        double s_dd;
        double d_d;

        Trajectory();
        Trajectory(Vehicle* vehicle, double s_d, double s_dd, double d_d, double d_dd);

        virtual ~Trajectory();

    private:
        double d_dd;

};

#endif
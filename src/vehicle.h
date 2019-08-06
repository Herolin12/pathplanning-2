#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <vector>
#include "mapping.h"
#include "behaviour.h"

class Vehicle
{
    public:
        int id;

        double x;
        double y;

        double vx;
        double vy;

        double s;
        double d;
        double yaw;

        // We can compute those
        int lane;
        double speed;
        double theta;
        State state;

        Mapping* map;

        Vehicle();
        Vehicle(int id, double x, double y, double vx, double vy, double s, double d, Mapping* map);
        Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double yaw, State state, Mapping* map);

        virtual ~Vehicle();

        // Returns a new vehicle at the next timestep
        Vehicle predict_position(double t);

        vector<Vehicle> ahead(vector<Vehicle> others);

        vector<Vehicle> behind(vector<Vehicle> others);

    private:
        double get_speed() const;
};

#endif
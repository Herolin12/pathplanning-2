#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <vector>
#include "constants.h"

class State;
class Vehicle;
class Mapping;

class Vehicle {
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
        vector<double> kinematics;

        State* state;
        Mapping* map;

        Vehicle();
        Vehicle(int id, double x, double y, double vx, double vy, double s, double d, Mapping* map);
        Vehicle(int id, double x, double y, double vx, double vy, double s, double d, double yaw,
                    vector<double> kinematics, State* state, Mapping* map);

        virtual ~Vehicle();

        // Returns a new vehicle at the next timestep
        Vehicle predict_position(double t);

        vector<Vehicle> ahead(vector<Vehicle> others, double T);

        vector<Vehicle> behind(vector<Vehicle> others, double T);

        vector<Vehicle> side(vector<Vehicle> others, double T, char mode);

        bool sort_distance_to_ego(const Vehicle& left, const Vehicle& right);

        bool sort_increment(const Vehicle& left, const Vehicle& right);

        bool sort_decrement(const Vehicle& left, const Vehicle& right);

    private:
        double get_speed() const;
};

#endif
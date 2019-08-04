#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <vector>
#include "mapping.h"

using namespace std;

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

        // We can compute those
        int lane;
        double speed;
        double theta;

        Mapping* map;

        /**
         * Constructors
         */
        Vehicle();
        Vehicle(int id, double x, double y, double vx, double vy, double s, double d, Mapping* map);

        /**
        * Destructor
        */
        virtual ~Vehicle();

        // Returns a new vehicle at the next timestep
        Vehicle predict_position(double t);

        /**
         * @brief Checks which vehicles are ahead of the current vehicle on the given lane
         * 
         * @param others the other vehicles which may be ahead
         * @param lane the lane to check against
         * @return vector<Vehicle>  the vehicles ahead of the current vehicle
         */
        vector<Vehicle> ahead(vector<Vehicle> others);

        /**
         * @brief Checks which vehicles are behind of the current vehicle on the given lane
         * 
         * @param others the other vehicles which may be behind
         * @param lane the lane to check against
         * @return vector<Vehicle>  the vehicles behind of the current vehicle
         */
        vector<Vehicle> behind(vector<Vehicle> others);

        
    private:
        double get_speed() const;
};
#endif
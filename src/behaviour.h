#include <iostream>
#include <math.h>
#include <string>
#include <vector>
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

class Behaviour {
    public:
        double s;
        double d;
        int lane;
        double ref_vel;
        string current_state;

        Behaviour(Vehicle ego, double ref_vel);

        vector<string> available_states()
}
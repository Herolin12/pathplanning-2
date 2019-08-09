#ifndef CONSTANTS
#define CONSTANTS

#define NUM_OF_PREVIOUS_POINTS 8
#define MAX_SPEED_MILES 49.5
#define MAX_SPEED_M MAX_SPEED_MILES*1.609*1000
#define MAX_SPEED_MS MAX_SPEED_MILES*0.44704
#define MAX_ACCELERATION 9.5
#define MAX_JERK 9.5
#define MIN_D 2
#define MAX_LANE_CHANGE_TIME 3
#define PATH_TIMESTEP 0.02
#define WAYPOINTS 5
#define INTERPOLATION_DISTANCE 0.6
#define PATH_WAYPOINTS_PER_SEC 50
#define TRACK_LENGTH 6945.554
#define LANE_WIDTH 4
#define BUFFER_RANGE 30
#define LANE_SPEED {MAX_SPEED_MILES*0.6, MAX_SPEED_MILES*0.8, MAX_SPEED_MILES}





#define VEHICLE_RADIUS 1.25              // meters
#define FOLLOW_DISTANCE 8.0              // distance to keep behind leading cars

#define PREVIOUS_PATH_POINTS_TO_KEEP 25
#define NUM_PATH_POINTS 50
#define PATH_DT 0.02                    // seconds

#define TRACK_LENGTH 6945.554           // meters

// number of waypoints to use for interpolation
#define NUM_WAYPOINTS_BEHIND 5
#define NUM_WAYPOINTS_AHEAD 5

// for trajectory generation/evaluation and non-ego car predictions
#define N_SAMPLES 20
#define DT 0.20                         // seconds

#define SPEED_LIMIT 21.5                // m/s
#define VELOCITY_INCREMENT_LIMIT 0.125

// cost function weights
#define COLLISION_COST_WEIGHT 99999
#define BUFFER_COST_WEIGHT 10
#define IN_LANE_BUFFER_COST_WEIGHT 1000
#define EFFICIENCY_COST_WEIGHT 10000
#define NOT_MIDDLE_LANE_COST_WEIGHT 100
// #define SPEED_LIMIT_COST_WEIGHT 9999
// #define MAX_ACCEL_COST_WEIGHT 9999
// #define AVG_ACCEL_COST_WEIGHT 1000
// #define MAX_JERK_COST_WEIGHT 9999
// #define AVG_JERK_COST_WEIGHT 1000
// #define TIME_DIFF_COST_WEIGHT 10
// #define TRAJ_DIFF_COST_WEIGHT 10

// DEPRECATED CONSTANTS
#define NUM_RANDOM_TRAJ_TO_GEN 4        // the number of perturbed trajectories to generate (for each perturbed duration)
#define NUM_TIMESTEPS_TO_PERTURB 2      // the number of timesteps, +/- target time, to perturb trajectories

// sigma values for perturbing targets
#define SIGMA_S 10.0                     // s
#define SIGMA_S_DOT 3.0                 // s_dot
#define SIGMA_S_DDOT 0.1                  // s
#define SIGMA_D 0                       // d
#define SIGMA_D_DOT 0                   // d_dot
#define SIGMA_D_DDOT 0                  // d_double_dot
#define SIGMA_T 0.05

#define MAX_INSTANTANEOUS_JERK 10       // m/s/s/s
#define MAX_INSTANTANEOUS_ACCEL 10      // m/s/s

#define EXPECTED_JERK_IN_ONE_SEC 2      // m/s/s
#define EXPECTED_ACC_IN_ONE_SEC 1       // m/s

#define PERCENT_V_DIFF_TO_MAKE_UP 0.5   // the percent difference between current velocity and target velocity to allow ego car to make up in a single trajectory  

#endif
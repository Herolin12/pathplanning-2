#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "mapping.cpp"
#include "vehicle.cpp"
#include "behaviour.cpp"
#include "trajectory.cpp"
#include "jmt.h"
#include "helpers.h"
#include "constants.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout;
using std::endl;
using Eigen::ArrayXd;
using Eigen::MatrixXd;
using namespace Eigen;

/* CONSTANTS defined over here */
// max speed limit in miles per sec
double ref_vel = 0.;
int lane = 1;

double distance_spacing;

double prev_x, prev_y, car_vx, car_vy, prev_vx, prev_vy, s, s_d, s_dd, d, d_d, d_dd;
vector<double> prev_speed;
Trajectory best_trajectory, prev_traj;
State state;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    // Interpolating the track points with spline
    double separation = INTERPOLATION_DISTANCE; // in m
    Mapping map = Mapping(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, separation, TRACK_LENGTH);
    vector<double> kinematics, vx, vy, previous_path_s, previous_path_d;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          int previous_size = previous_path_x.size();
          vector<double> points_x, points_y;

          if (previous_size < 2){
            car_vx = 0;
            car_vy = 0;
            prev_vx = 0;
            prev_vy = 0;
            prev_speed = {0, 0};
            prev_x = car_x - cos(car_yaw);
            prev_y = car_y - sin(car_yaw);
            points_x.push_back(prev_x);
            points_y.push_back(prev_y);
            points_x.push_back(car_x);
            points_y.push_back(car_y);
            state = State("RDY", d2lane(car_d));
            double lane = state.current_lane;
            vx = {prev_vx, car_vx};
            vy = {prev_vy, car_vy};
            //kinematics = get_kinematics(points_x, points_y, car_s, car_d, vx, vy, prev_speed, map);
            kinematics = {car_s, 0, 0, lane, 0, 0};
          }
          else {
            // Put current internal kinematics to rely on previous points
            // Consider only 10 previous points here
            int size = NUM_OF_PREVIOUS_POINTS;
            int first_prev_point = previous_size - size;
            prev_traj = best_trajectory;
            for (int i = 0; i < size; i++){
              points_x.push_back(previous_path_x[previous_size-1-i]);
              points_y.push_back(previous_path_y[previous_size-1-i]);
            }
            car_vx = (points_x[size-1] - points_x[0])/ (size*PATH_TIMESTEP);
            car_vy = (points_y[size-1] - points_y[0])/ (size*PATH_TIMESTEP);
            
            // Calculate speed from the collected previous points x, y
            // Use this 10 points to find previous vx, vy, and speed (s_d, d_d)
            prev_vx = calculate_speed(points_x);
            prev_vy = calculate_speed(points_y);
            previous_path_s = prev_traj.kinematics_s.back();
            previous_path_d = prev_traj.kinematics_d.back();
            kinematics = {previous_path_s[0], previous_path_s[1], previous_path_s[2],
                          previous_path_d[0], previous_path_d[1], previous_path_d[2]};
          }

          //cout << "=========================================================" << endl;
          /*
          cout << "car_x | " << car_x << endl;
          cout << "car_y | " << car_y << endl;
          cout << "car_s | " << car_s << endl;
          cout << "car_d | " << car_d << endl;
          cout << "yaw   | " << car_yaw << endl;
          cout << "speed | " << car_speed << endl;
          cout << "end_s | " << end_path_s << endl;
          cout << "end_d | " << end_path_d << endl;
          cout << "psize | " << previous_size << endl;
          cout << "car_vx| " << car_vx << endl;
          cout << "car_vy| " << car_vy << endl;
           */
          
          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

          // Transform sensor fusion into vehicle object
          Vehicle ego = Vehicle(000, car_x, car_y, car_vx, car_vy, car_s, car_d, car_yaw,
                                      kinematics, &state, &map);
          Behaviour planner(&ego, ref_vel);

          vector<Vehicle> surrounding_vehicles;
          for (int i = 0; i < sensor_fusion.size(); i++){
            int id = sensor_fusion[i][0];
            double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];
            Vehicle v = Vehicle(id, x, y, vx, vy, s, d, &map);
            surrounding_vehicles.push_back(v);
          }

          vector<Trajectory> trajectories = planner.generate_trajectory(points_x, points_y);
          best_trajectory = planner.get_best_trajectory(trajectories, surrounding_vehicles);

          vector<Vehicle> vehicles_ahead = ego.ahead(surrounding_vehicles);
          Vehicle car_ahead;

          // For every 0.02 sec blocks, max acceleration/ displacement is 0.004m
          double new_s = car_s;
          double new_d, new_x, new_y;
          bool slow_down;
          vector<double> xy;
          distance_spacing = 0.224;

          if ((!vehicles_ahead.empty()) && ((car_ahead.s - ego.s) <= BUFFER_RANGE)){
            slow_down = true;
          }
          else {
            slow_down = false;
          }

          if (slow_down){
            ref_vel -= distance_spacing;
          }
          else if (ref_vel <= MAX_SPEED_MILES){
            ref_vel += distance_spacing;
          }

          double increment = .02*ref_vel/2.24;
          for (int i = 0; i < 50; i++) {
            new_s += increment;
            xy = map.getXY(new_s, 6);

            //next_x_vals.push_back(xy[0]);
            //next_y_vals.push_back(xy[1]);
          }
          next_x_vals = best_trajectory.x;
          next_y_vals = best_trajectory.y;

          cout << "=========================================================" << endl;
          cout << "Dist  | " << best_trajectory.kinematics_s.back()[0] << endl;
          cout << "Speed | " << best_trajectory.kinematics_s.back()[1] << endl;
          cout << "Acc   | " << best_trajectory.kinematics_s.back()[2] << endl;
          cout << "Jerk  | " << best_trajectory.kinematics_s.back()[3] << endl;
          
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
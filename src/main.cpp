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
#include "helpers.h"
#include "constants.h"
#include "jmt.h"
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
int gg = 0;

double distance_spacing;

vector <double> previous_path_s;
vector <double> previous_path_d;

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
    double separation = 0.5; // in m
    Mapping map = Mapping(map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, INTERPOLATION_DISTANCE, TRACK_LENGTH);

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

          for (int i = 0; i < PREVIOUS_POINTS_USED; i++){
            points_x.push_back(previous_path_x[i]);
            points_y.push_back(previous_path_y[i]);
          }


          double car_vx, car_vy;
          double px1, px2, py1, py2;
          if (previous_size >= 2){
            px1 = previous_path_x[previous_size-1];
            py1 = previous_path_y[previous_size-1];
            px2 = previous_path_x[0];
            py2 = previous_path_y[0];
            car_vx = (px1 - px2)/(previous_size*PATH_TIMESTEP);
            car_vy = (py1 - py2)/(previous_size*PATH_TIMESTEP);
          }
          else {
            car_vx = 0;
            car_vy = 0;
          }

          cout << "=========================================================" << endl;
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
          State state;
          Vehicle ego = Vehicle(000, car_x, car_y, car_vx, car_vy, car_s, car_d, car_yaw, state, &map);

          vector<Vehicle> surrounding_vehicles;
          for (int i = 0; i < sensor_fusion.size(); i++){
            int id = sensor_fusion[i][0];
            double x = sensor_fusion[i][1];
            double y = sensor_fusion[i][2];
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double s = sensor_fusion[i][5];
            double d = sensor_fusion[i][6];
            State state;
            Vehicle v = Vehicle(id, x, y, vx, vy, s, d, &map);
            surrounding_vehicles.push_back(v);
          }
          vector<Vehicle> vehicles_ahead = ego.ahead(surrounding_vehicles);
          Vehicle car_ahead;

          if (!vehicles_ahead.empty()){
            car_ahead = vehicles_ahead[0];
            cout << "ahead | " << car_ahead.s - ego.s << endl;
          }


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
          cout << "incre | " << increment << endl;
          for (int i = 0; i < 50; i++) {
            new_s += increment;
            xy = map.getXY(new_s, 6);
            
            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
          }

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
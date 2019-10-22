#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

#include "spline.h"

#include "vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

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

  Vehicle my_car;

  my_car.init(0.0, 0.0,0.0, 0.0, 0.0, 0.0);

  // Start of the main loop
  h.onMessage([&my_car, 
               &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        /**
         * Telemetry JSON structure:
         * 
         * ["telemetry",
         *   { "d":6.164833,
         *     "end_path_d":0,
         *     "end_path_s":0,
         *     "previous_path_x":[],
         *     "previous_path_y":[],
         *     "s":124.8336,
         *     "sensor_fusion":[[0,1028.854,1148.57,40.37725,16.71837,243.9722,9.999819],
         *                       ......
         *                      [11,762.1,1441.7,0,0,6653.453,-280.8947]],
         *     "speed":0,
         *     "x":909.48,
         *     y":1128.67,
         *     "yaw":0}]
        **/

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

          /**
           * Sensor fusion data
           * 0: Car unique ID
           * 1: map coordinate x
           * 2: map coordinate y
           * 3: vx
           * 4: vy
           * 5: s
           * 6: d
           */
          
          // This limits the size of data to be reused to two
          int prev_size = std::fmin(previous_path_x.size(),2);
          // int prev_size = previous_path_x.size();

          if (prev_size > 2){
            // Recalculate the data for the car, assuming that it is 2 points ahead
            car_x = previous_path_x[prev_size-1];
            car_y = previous_path_y[prev_size-1];
            double car_x_prev = previous_path_x[prev_size-2];
            double car_y_prev = previous_path_y[prev_size-2];
            car_yaw = atan2(car_y - car_y_prev, car_x - car_x_prev);
            vector<double> end_path_sd = getFrenet(car_x, car_y, car_yaw, 
                                                  map_waypoints_x, map_waypoints_y);
            car_s = end_path_sd[0];
            car_d = end_path_sd[1];
          }

          // int prev_size = previous_path_x.size();
          // if (prev_size > 0) {
          //   car_s = end_path_s;
          //   car_d = end_path_d;
          // }

          // These two vectors save information about the closest cars in all three lanes 
          // with their relative distance and approaching speed. This information will
          // be used in cost functions to make decision on actions to be taken.
          // Note that all values are positive.
          
          // cars behind that will not cause danger are away and have same speed.
          vector<vector<float>> approaching_cars_behind(3,{1.0e15,0.0});
          // cars behind that will not cause danger are away and have same speed.
          vector<vector<float>> approaching_cars_ahead(3,{1.0e15,0.0});

          // For each lane, find the closest 
          for (int i=0; i < sensor_fusion.size(); i++){
            float d = sensor_fusion[i][6];
            int l = (int) std::floor(d/4.0);
            if ((d > 0) && (l > -1) && (l < 3)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_car_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              // Simple predictions of the position at the next step
              // Influence of acceleration on the decision is neglected as it
              // will be proportional to 0.02*0.02 = 0.004
              //check_car_s += ((double) 0.02*check_speed);
              float new_car_s = car_s + car_speed * 0.02;
              float new_check_car_s = check_car_s + check_car_speed * 0.02;

              double distance = new_check_car_s - new_car_s;

              //Look at cars around. Treat a car at the same s as one behind.
              if (distance > 0){
                if (distance < approaching_cars_ahead[l][0]){
                  approaching_cars_ahead[l][0] = distance;
                  approaching_cars_ahead[l][1] = check_car_speed * 2.24;
                }
              }
              else {
                distance *= -1;
                if (distance < approaching_cars_behind[l][0]){
                  approaching_cars_behind[l][0] = distance;
                  approaching_cars_behind[l][1] = check_car_speed * 2.24;
                }
              }
            }
          }

          my_car.update(car_x, car_y, car_speed, car_yaw, car_s, car_d,
                     approaching_cars_behind, approaching_cars_ahead);

          my_car.set_next_lane();

          vector<double> ptss;
          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // Add Two points in the vector so that the speed and acceleration will be continuous
          if (prev_size < 2) {
            // This will be executed when the path is calculated first time. The first point is backwards in 
            // right direction.
            ref_x = car_x;
            ref_y = car_y;
            ref_yaw = deg2rad(car_yaw);

            double prev_car_x = car_x - cos(ref_yaw);
            double prev_car_y = car_y - sin(ref_yaw);

            double ds = sqrt((car_x-prev_car_x)*(car_x-prev_car_x) +
                              (car_y-prev_car_y)*(car_y-prev_car_y));

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

            ptss.push_back(car_s - ds);
            ptss.push_back(car_s);
          }
          else {
            // Two last points in the previous path are added to the new path.
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size -2];
            double ref_y_prev = previous_path_y[prev_size -2];

            double ds = sqrt((car_x-ref_x_prev)*(car_x-ref_x_prev) +
                              (car_y-ref_y_prev)*(car_y-ref_y_prev));

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

            ptss.push_back(car_s - ds);
            ptss.push_back(car_s);

          }

          // The decision to change lane or not is taken. The end points are calculated
          // and used to create the spline function.

          // Add three points at the end of the path so that it will become a stable
          // path in the middle of the lane. Here, the lane could be different than
          // the existing one.

          // Transform to XY coordinate and add them to the pionts for spline
          
          float lane_change_distance = my_car.lane_change_distance;
          vector<double> nextwp0 = getXY(car_s + 1.0*lane_change_distance, 2 + 4.0*my_car.target_lane, 
                                map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextwp1 = getXY(car_s + 2.0*lane_change_distance, 2 + 4.0*my_car.target_lane, 
                                map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> nextwp2 = getXY(car_s + 3.0*lane_change_distance, 2 + 4.0*my_car.target_lane, 
                                map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(nextwp0[0]);
          ptsx.push_back(nextwp1[0]);
          ptsx.push_back(nextwp2[0]);

          ptsy.push_back(nextwp0[1]);
          ptsy.push_back(nextwp1[1]);
          ptsy.push_back(nextwp2[1]);

          ptss.push_back(car_s + 1.0*lane_change_distance);
          ptss.push_back(car_s + 2.0*lane_change_distance);
          ptss.push_back(car_s + 3.0*lane_change_distance);

          tk::spline sx;
          tk::spline sy;

          sx.set_points(ptss, ptsx);
          sy.set_points(ptss, ptsy);

          // Add points from existing path first:
          for (int i = 0; i < prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // Discretize the path so that the distance becomes compatible
          // Use spline and interpolate points and add to the new calculated path
          // with the reference velocity.
          double s_point = car_s;

          my_car.cruise_control();

          for (int i = 1; i <= 50 - prev_size; i++){
            s_point += my_car.ref_speed*0.02/2.24;
            next_x_vals.push_back(sx(s_point));
            next_y_vals.push_back(sy(s_point));
          }

          // End TODO

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
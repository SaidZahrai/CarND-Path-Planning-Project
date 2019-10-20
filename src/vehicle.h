#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();

  // Destructor
  virtual ~Vehicle();

  void Init(float x_, float y_, float vx_, float vy_, float s_, float d_);

  // Vehicle functions
  void Update(float x_, float y_, float vx_, float vy_, float s_, float d_,
              vector<vector<float>> approaching_cars_behind_,
              vector<vector<float>> approaching_cars_ahead);

  void set_next_lane();

  void accelerate(double to_speed);

  void decelerate(double to_speed);

  void cruise_control();

  vector<string> successor_states();

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1},
                                     {"LCR", -1}, {"PLCR", -1}};

  float maximum_acceleration;
  float comfortable_acceleration;
  float maximum_deceleration;
  float comfortable_deceleration;

  int current_lane;
  int target_lane;
  int previous_lane;

  float ref_speed;

  float ref_acc;

  float x;
  float y;
  float speed;
  float yaw;

  float s;
  float d;
  float a;

  string state;

  vector<vector<float>> approaching_cars_behind;
  vector<vector<float>> approaching_cars_ahead; 

  float minimum_safe_distance;
  float lane_change_distance;

  float maximum_speed;  
  float maximum_acc;
  float maximum_jerk;
};

#endif  // VEHICLE_H
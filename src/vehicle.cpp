#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <iostream>
#include <vector>
#include "cost.h"

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

void Vehicle::init(float x_, float y_, float speed_, float yaw_, float s_, float d_){
  x = x_;
  y = y_;
  speed = speed_;
  yaw = yaw_;
  s = s_;
  d = d_;

  current_lane = (int) floor(d / 4.0);

  previous_lane = current_lane;
  target_lane = current_lane;

  float max_speed;
  float max_braking_distance;

  max_speed = 50.0*1.609344*1000.0/3600.0; // max_speed is set to 50 MPH
  max_braking_distance = 200.0; // maximum distance for stopping from maximum speed

  maximum_acceleration = 10; // 100 kph per 10 seconds
  float brake_time = max_braking_distance / (max_speed/2.0); // Constant deceleration
  maximum_deceleration = max_speed / brake_time; 
  comfortable_acceleration = maximum_acceleration/2.0;
  comfortable_deceleration = maximum_deceleration;

  minimum_safe_distance = 50.0;
  lane_change_distance = 30.0;

  maximum_speed = 50.0 * 0.95;
  maximum_jerk = 10.0;

  vector<vector<float>> approaching_cars(3,{minimum_safe_distance,0.0});
  approaching_cars_ahead = approaching_cars;
  approaching_cars_behind = approaching_cars;

  state = "KL";
  ref_speed = 0.0;
  ref_acc = 0.0;
}

Vehicle::~Vehicle() {}

void Vehicle::update(float x_, float y_, float speed_, float yaw_, float s_, float d_,
                     vector<vector<float>> approaching_cars_behind_,
                     vector<vector<float>> approaching_cars_ahead_){
  x = x_;
  y = y_;
  speed = speed_;
  yaw = yaw_;
  s = s_;
  d = d_;

  current_lane = (int) floor(d / 4.0);

  approaching_cars_behind = approaching_cars_behind_;
  approaching_cars_ahead = approaching_cars_ahead_;
}

void Vehicle::accelerate(double to_speed){
  if (ref_speed < to_speed) {
    ref_speed += comfortable_acceleration * 0.02 / 1609.344 * 3600;
  }
}

void Vehicle::decelerate(double to_speed, double rel_distance){
  if (ref_speed > to_speed) {
    // Acceleration is increase if a car is too close
    ref_speed += -comfortable_deceleration * 0.02 / 1609.344 * 3600 / std::fmax(0.1,rel_distance/minimum_safe_distance);
  }
}

void Vehicle::cruise_control(){
  double acceleration; 
  if (approaching_cars_ahead[current_lane][0] <= minimum_safe_distance){
    decelerate(approaching_cars_ahead[current_lane][1], approaching_cars_ahead[current_lane][0]);
  } else {
    accelerate(maximum_speed);
  }
}

vector<string> Vehicle::successor_states() {
  // Simple FSM with three states: Keep Lane, Lane Change Left, Lane Chenge Right
  // There is also a cruise control that avoids collisions to the car in the front
  // "CXL" and "CXR" are the states that lane changes are happening.
  vector<string> states;

  string state = this->state;
  if(state.compare("KL") == 0) {
    // The car is drivin in a lane. See if it is better to change the lane.
    states.push_back("KL");
    if (current_lane > 0) states.push_back("CLL");
    if (current_lane < 2) states.push_back("CLR");
  } 
  else if (state.compare("CXX") == 0) {
    if ((d > target_lane * 4 + 1.5) && (d < (target_lane + 1)* 4 - 1.5)) {
      // Target lane is reached. Change the state to Keep Lane
      state = "KL";
      states.push_back("KL");
      current_lane = target_lane;
      std::cout << "Lane change complete to " << target_lane << std::endl;
    }
    else {
      // Continue lane changing.
      states.push_back("CXX");
    }
  } 
  return states;
}


void Vehicle::set_next_lane() {
  vector<string> possible_states = successor_states();
  float lowest_cost = 1.0e15;
  float cost = 0.0;
  int next_lane = current_lane;
  int selected = 0;
  string next_state = "KL";
  for (int s=0; s < possible_states.size(); s++){
    if (possible_states[s].compare("KL") == 0){
      // Keep Lane
      next_lane = current_lane;
      next_state = "KL";
      cost = calculate_cost(this, current_lane, next_lane);
    }
    else if (possible_states[s].compare("CLL") == 0){
      // Start to change the lane to left. If this is chosen, next state will be Lane Changing.
      next_lane = current_lane-1;
      next_state = "CXX";
      cost = calculate_cost(this, current_lane, next_lane);
    }
    else if (possible_states[s].compare("CLR") == 0){
      // Start to change the lane to right. If this is chosen, next state will be Lane Changing.
      next_lane = current_lane + 1;
      next_state = "CXX";
      cost = calculate_cost(this, current_lane, next_lane);
    }
    else if (possible_states[s].compare("CXX") == 0){
      // Lane changing is ongoing. Keep the target!
      next_lane = target_lane;
      next_state = "CXX";
      cost = 0;
    }
    if (cost < lowest_cost){
      lowest_cost = cost;
      previous_lane = current_lane;
      target_lane = next_lane;
      state = next_state;
      selected = s;
    }
  }
  // std::cout << "Best lane " << target_lane << " cost: " 
  // << lowest_cost << " selected: " << possible_states[selected] 
  // << " End state: " << state << std::endl;
}

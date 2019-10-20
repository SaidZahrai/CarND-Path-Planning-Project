#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"
#include <iostream>

using std::string;
using std::vector;

float lane_cost(const Vehicle *vehicle,
                        const int current_lane, const int next_lane) {
  float cost = 1.0;
  if (next_lane == 1) cost = 0.0;
  return cost;
}

float right_passage_cost(const Vehicle *vehicle,
                        const int current_lane, const int next_lane) {
  float cost = 0.0;
  if ((next_lane > current_lane) && 
      (vehicle->approaching_cars_ahead[current_lane][0] <= vehicle->minimum_safe_distance) &&
      (vehicle->approaching_cars_ahead[current_lane][1] <= vehicle->speed)) cost = 1.0;
  return cost;
}

float inefficiency_cost(const Vehicle *vehicle,
                        const int current_lane, const int next_lane) {
  float cost = 0.0;
  if (vehicle->approaching_cars_ahead[next_lane][0] <= vehicle->minimum_safe_distance)
        cost = 1.0 - std::pow(std::fmin(vehicle->approaching_cars_ahead[next_lane][1]/vehicle->maximum_speed , 1.0),4.0);
  return cost;
}

float collision_cost(const Vehicle *vehicle,
                        const int current_lane, const int next_lane) {
  float cost = 0.0;
  if ((current_lane != next_lane)
      && (
          (vehicle->approaching_cars_behind[next_lane][0] < vehicle->minimum_safe_distance) ||
          (vehicle->approaching_cars_ahead[next_lane][0] < vehicle->minimum_safe_distance)
           )){
        cost = 1;
      }
  return cost;
}

// float collision_cost(const Vehicle *vehicle,
//                         const int current_lane, const int next_lane) {
//   float cost = 0.0;
//   if ((current_lane != next_lane)
//       && (
//           ((vehicle->approaching_cars_behind[next_lane][0] < vehicle->minimum_safe_distance) &&
//            (vehicle->approaching_cars_behind[next_lane][1] > 0.8 * vehicle->speed))
//        || ((vehicle->approaching_cars_ahead[next_lane][0] < vehicle->minimum_safe_distance) &&
//            (vehicle->approaching_cars_ahead[next_lane][1] < 1.2 * vehicle->speed))
//            )){
//         cost = 1;
//       }
//   return cost;
// }

float calculate_cost(const Vehicle *vehicle,
                     const int current_lane, const int next_lane) {
  // Sum weighted cost functions to get total cost for trajectory.

  float cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<float(const Vehicle *, const int, const int)
    >> cf_list = {lane_cost, inefficiency_cost, right_passage_cost, collision_cost};
  vector<float> weight_list = {0.5, 4.0, 0.25, 10.0};
  for (int i = 0; i < cf_list.size(); ++i) {
    cost += weight_list[i]*cf_list[i](vehicle, current_lane, next_lane);
  }
  return cost;
}

#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

float calculate_cost(const Vehicle *vehicle,const int current_lane, const int next_lane);

float lane_cost(const Vehicle *vehicle,const int current_lane, const int next_lane);

float right_passage_cost(const Vehicle *vehicle,const int current_lane, const int next_lane);

float collision_cost(const Vehicle *vehicle,const int current_lane, const int next_lane);

float inefficiency_cost(const Vehicle *vehicle,const int current_lane, const int next_lane);

#endif  // COST_H
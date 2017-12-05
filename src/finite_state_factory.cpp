#include "finite_state_factory.h"
using std::unique_ptr;
#include <algorithm>
#include "lane_keep.h"
#include "prepare_lane_change.h"
#include "lane_change_left.h"
#include "lane_change_right.h"
#include "lane.h"

unique_ptr<State> FiniteStateFactory::GetBestState(LocalizationData& localization_data, vector<vector<double>>& sensor_fusion){
  vector<unique_ptr<State>> states;
  states.push_back(unique_ptr<State>(new LaneKeep(localization_data)));
  states.push_back(unique_ptr<State>(new PrepareLaneChange(localization_data)));
  states.push_back(unique_ptr<State>(new LaneChangeLeft(localization_data)));
  states.push_back(unique_ptr<State>(new LaneChangeRight(localization_data)));

  vector<double> state_costs;
  for (const auto& state : states) {
    double cost = CalculateCost(state.get(), localization_data, sensor_fusion);
    state_costs.push_back(cost);
  }

  auto best_index = std::distance(state_costs.begin(), std::min_element(state_costs.begin(), state_costs.end()));
  return move(states[best_index]);
}

double FiniteStateFactory::CalculateCost(State* state, LocalizationData& localization_data, vector<vector<double>>& sensor_fusion) {
  double cost = 0;
  cost += OffroadCost(state, 1000);
  cost += LaneChangeCost(state, localization_data, .06);
  cost += OtherCarInLaneCost(state, sensor_fusion, 150); 
  cost += SpeedCost(state, localization_data, .0001);
  return cost;
}

double FiniteStateFactory::OffroadCost(State* state, double weight) {
  const double kLaneMargin = 2;
  if (state->TargetD() > Lane::right_lane + kLaneMargin || state->TargetD() < Lane::left_lane - kLaneMargin) {
    return weight;
  }
  return 0;
}

double FiniteStateFactory::LaneChangeCost(State* state, LocalizationData& localization_data, double weight) {
  if (localization_data.d != state->TargetD()) {
    return weight;
  }
  return 0;
}

double FiniteStateFactory::OtherCarInLaneCost(State* state, vector<vector<double>>& sensor_fusion, double weight) {
  vector<double> other_car_squared_distances;

  for (auto car : sensor_fusion) {
    double other_car_d = car[6];
    const double kLaneMargin = 2;
    bool in_lane = other_car_d > state->TargetD() - kLaneMargin  && other_car_d < state->TargetD() + kLaneMargin; 
    if (!in_lane)
      continue;

    double other_car_s = car[5];
    double other_car_distance = other_car_s - state->ProjectedS();
    const double safe_passing_distance = -6; 
    if (other_car_distance >= safe_passing_distance) {
      other_car_squared_distances.push_back(pow(other_car_distance, 2)); 
    }
  }

  if (other_car_squared_distances.size() > 0) {
    double squared_distance_to_nearest_car = *std::min_element(other_car_squared_distances.begin(), other_car_squared_distances.end());
    return weight / squared_distance_to_nearest_car;
  }
  return 0;
}

double FiniteStateFactory::SpeedCost(State* state, LocalizationData& localization_data, double weight) {
  double speed_delta = localization_data.speed_limit - state->TargetSpeed();
  return weight * pow(speed_delta, 2);
}
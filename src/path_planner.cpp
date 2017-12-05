#include "path_planner.h"
#include <iostream>
#include <memory>
#include <math.h>
using std::vector;
#include "frenet_cartesian_converter.h"
#include "state.h"
#include "localization_data.h"
#include "finite_state_factory.h"
#include "lane.h"

PathPlanner::PathPlanner(const FrenetCartesianConverter& frenet_cartesian_converter) :
  frenet_cartesian_converter_(frenet_cartesian_converter),
  reference_speed_(0), 
  lane_(middle_lane) {}

PathPlanner::~PathPlanner() {}

void PathPlanner::Fit(LocalizationData& localization_data, vector<vector<double>>& sensor_fusion, const vector<double>& previous_path_x, const vector<double>& previous_path_y) {
  localization_data.d = lane_; // The simulator seems to give gabage data for d
  localization_data.speed = reference_speed_;
  std::unique_ptr<State> state = FiniteStateFactory::GetBestState(localization_data, sensor_fusion);
  reference_speed_ = state->TargetSpeed();
  lane_ = state->TargetD();

  reference_x_ = localization_data.x;
  reference_y_ = localization_data.y;
  reference_angle_ = localization_data.yaw * M_PI/180;

  SetSplineAnchorPoints(localization_data, previous_path_x, previous_path_y);
  SetNextPath(previous_path_x, previous_path_y);
}

vector<double> PathPlanner::GetNextXValues() {
  return next_x_vals_;
}

vector<double> PathPlanner::GetNextYValues() {
  return next_y_vals_;
}

void PathPlanner::SetSplineAnchorPoints(LocalizationData& localization_data, const vector<double>& previous_path_x, const vector<double>& previous_path_y) {
  vector<double> spline_anchor_points_x;
  vector<double> spline_anchor_points_y;
  int previous_size = previous_path_x.size();
  double car_s = localization_data.s;
  if (previous_size> 0) {
    car_s = localization_data.end_path_s;
  }

  if(previous_size < 2) {
    double previous_x = localization_data.x - cos(localization_data.yaw);
    double previous_y = localization_data.y - sin(localization_data.yaw);  
    spline_anchor_points_x.push_back(previous_x);
    spline_anchor_points_x.push_back(localization_data.x);  
    spline_anchor_points_y.push_back(previous_y);
    spline_anchor_points_y.push_back(localization_data.y);
  }
  else {
    reference_x_ = previous_path_x[previous_size-1];
    reference_y_ = previous_path_y[previous_size-1];  
    double previous_reference_x = previous_path_x[previous_size-2];
    double previous_reference_y = previous_path_y[previous_size-2];  
    reference_angle_ = atan2(reference_y_-previous_reference_y,reference_x_-previous_reference_x);  
    spline_anchor_points_x.push_back(previous_reference_x);
    spline_anchor_points_x.push_back(reference_x_);  
    spline_anchor_points_y.push_back(previous_reference_y);
    spline_anchor_points_y.push_back(reference_y_);
  }  

  const int kAnchorPointCount = 3;
  const int kAnchorPointSpacing = 30;
  for (int i = 1; i <= kAnchorPointCount; i++) {
    vector<double> anchor_point = frenet_cartesian_converter_.FrenetToCartesian(car_s + i * kAnchorPointSpacing, lane_);
    spline_anchor_points_x.push_back(anchor_point[0]);
    spline_anchor_points_y.push_back(anchor_point[1]);
  }  

  for(int i = 0; i < spline_anchor_points_x.size() ; i++) {
    double shift_x = spline_anchor_points_x[i] - reference_x_;
    double shift_y = spline_anchor_points_y[i] - reference_y_;  
    spline_anchor_points_x[i] = (shift_x * cos(-reference_angle_) - shift_y * sin(-reference_angle_));
    spline_anchor_points_y[i] = (shift_x * sin(-reference_angle_) + shift_y * cos(-reference_angle_));
  }

  spline_.set_points(spline_anchor_points_x,spline_anchor_points_y);
}

void PathPlanner::SetNextPath(const vector<double>& previous_path_x, const vector<double>& previous_path_y) {
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  int previous_size = previous_path_x.size();
  for(int i = 0; i < previous_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  const double target_x = 30.0;
  double target_y = spline_(target_x);

  double target_distance = sqrt((target_x * target_x) + (target_y * target_y));
  double x_add_on = 0;

  const double dist_inc = 0.44;
  const double kSecondsPerFrame = 0.02;
  const double kMphToMps = 0.44704;
  for(int i = 1; i < 50-previous_size; i++) {
    double N = target_distance / (kSecondsPerFrame * reference_speed_ * kMphToMps);

    double x_point = x_add_on + target_x / N;
    double y_point = spline_(x_point);

    x_add_on = x_point;

    double updated_x_point = x_point * cos(reference_angle_) - y_point * sin(reference_angle_);
    double updated_y_point = x_point * sin(reference_angle_) + y_point * cos(reference_angle_);

    x_point = updated_x_point + reference_x_;
    y_point = updated_y_point + reference_y_;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
  next_x_vals_ = next_x_vals;
  next_y_vals_ = next_y_vals;
}


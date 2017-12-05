#ifndef PathPlanner_H
#define PathPlanner_H
#include <memory>
#include <vector>
#include "spline.h"
using std::vector;
#include "frenet_cartesian_converter.h"
#include "state.h"
#include "localization_data.h"
#include "lane.h"


class PathPlanner {
public:
  PathPlanner(const FrenetCartesianConverter& frenet_cartesian_converter);
  virtual ~PathPlanner();

  void Fit(LocalizationData& localization_data, vector<vector<double>>& sensor_fusion, const vector<double>& previous_path_x, const vector<double>& previous_path_y);
  vector<double> GetNextXValues();
  vector<double> GetNextYValues();

private:
  FrenetCartesianConverter frenet_cartesian_converter_;
  vector<double> next_x_vals_;
  vector<double> next_y_vals_;
  double reference_speed_;
  double reference_x_;
  double reference_y_;
  double reference_angle_;
  int lane_;
  tk::spline spline_;
  void SetSplineAnchorPoints(LocalizationData& localization_data, const vector<double>& previous_path_x, const vector<double>& previous_path_y);
  void SetNextPath(const vector<double>& previous_path_x, const vector<double>& previous_path_y);
};
#endif /* PathPlanner_H */
